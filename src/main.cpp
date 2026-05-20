#include <Arduino.h>
#include <LittleFS.h>
#include <M5Unified.h>
#include <SPI.h>
#include <cstdio>
#include <esp_dmx.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <lvgl.h>

/* =========================================================================
 * 1. PIN MAP  (M5Stack CoreS3 SE + Base DMX)
 * ========================================================================= */

#define DMX_TX_PIN 7
#define DMX_RX_PIN 10
#define DMX_EN_PIN 6
#define DMX_PORT 1

#include "hal/uart_ll.h"

// Manual DMX TX — bypasses esp_dmx's broken gptimer-ISR path on ESP32-S3/IDF5.
// Called only from dmxTask (Core 1), so the busy-spin does not affect the UI.
//
// SAFETY NOTE: ets_delay_us() disables the CPU cache on the calling core while
// spinning. This function must NEVER be called while another core is attempting
// to suspend this task or perform any operation that touches the scheduler
// context of this core. vTaskSuspend() from Core 0 targeting a Core 1 task
// that is inside ets_delay_us() = guaranteed EXCCAUSE 7 cache panic.
// The only safe cross-core flash serialisation primitive is
// dmx_driver_disable(), which uses esp_intr_disable() internally and is
// cache-safe.
static void dmx_send_manual(const uint8_t *data, size_t size) {
  uart_dev_t *dev = UART_LL_GET_HW(DMX_PORT);

  // EN pin is wired through the GPIO matrix as UART RTS.
  // sw_rts=0 → RTS LOW → transceiver in TX mode.
  dev->conf0.sw_rts = 0;

  // BREAK: TX line forced LOW for ≥176 µs
  dev->conf0.txd_inv = 1;
  ets_delay_us(176);

  // MAB: TX line HIGH for ≥12 µs
  dev->conf0.txd_inv = 0;
  ets_delay_us(12);

  // Flush TX FIFO, then stream bytes
  uart_ll_txfifo_rst(dev);
  for (size_t i = 0; i < size; i++) {
    while (uart_ll_get_txfifo_len(dev) == 0) { /* wait for FIFO space */
    }
    uart_ll_write_txfifo(dev, &data[i], 1);
  }

  // Wait for shift register to drain
  while (uart_ll_get_txfifo_len(dev) < 128) {
    ets_delay_us(10);
  }
  ets_delay_us(50);
}

/* =========================================================================
 * 2. CONSTANTS
 * ========================================================================= */

#define SECURITY_PIN "0000"
#define NUM_SCENES 8
#define MAX_RECORD_SEC 20
#define DMX_FRAME_RATE 44
#define MAX_FRAMES (MAX_RECORD_SEC * DMX_FRAME_RATE) // 880 frames
#define DMX_PACKET_SIZE 513

/* =========================================================================
 * 3. GLOBAL STATE
 * ========================================================================= */

// Written by dmxTask (Core 1), read by loop() (Core 0) — needs volatile.
volatile uint32_t lastDmxRxTime = 0;
volatile uint16_t activeFrameCount = 0;
// Shared between dmxTask command handling and the play loop on the same task,
// also read by loop() for display — volatile is sufficient, no mutex needed
// because loop() only reads and dmxTask is the sole writer.
volatile uint16_t activeFrameInterval = 22;

lv_obj_t *dmxInLabel = nullptr;
lv_obj_t *tempLabel = nullptr;

enum SlotMenuMode { SLOT_MENU_PLAY, SLOT_MENU_REC, SLOT_MENU_CLEAR };

enum SystemState {
  STATE_PIN_LOCK,
  STATE_SETTINGS_MENU,
  STATE_SLOT_SELECT_REC,
  STATE_SLOT_SELECT_PLAY,
  STATE_SLOT_SELECT_CLEAR,
  STATE_REC_MODE_SELECT,
  STATE_RECORDING_STATIC,
  STATE_RECORDING_LOOP,
  STATE_PLAYING_SCENE,
  STATE_LOADING,
  STATE_SAVING,
  STATE_DMX_INPUT_ALERT
};

SystemState currentState = STATE_SLOT_SELECT_PLAY;
String inputPin = "";
uint8_t selectedSlot = 0;
uint8_t activeMode = 0;

// Frame buffer — allocated in PSRAM at startup, never freed.
uint8_t **dmxBuffer = NULL;

struct __attribute__((packed)) SceneFileHeader {
  uint8_t isLoop;
  uint16_t frames;
  uint16_t frameIntervalMs;
};

/* -------------------------------------------------------------------------
 * FreeRTOS inter-task messaging
 * ---------------------------------------------------------------------- */

enum DmxCmdType {
  DMX_CMD_IDLE,
  DMX_CMD_PLAY,
  DMX_CMD_REC_STATIC,
  DMX_CMD_REC_LOOP,
  DMX_CMD_STOP,
  DMX_CMD_UPDATE_INTERVAL
};

struct DmxCommand {
  DmxCmdType type;
  uint8_t mode; // 0=static, 1=loop
  uint16_t frameCount;
  uint16_t interval;
};

enum FsCmdType { FS_CMD_SAVE, FS_CMD_LOAD, FS_CMD_SCAN_SLOTS, FS_CMD_DELETE };

struct FsCommand {
  FsCmdType type;
  uint8_t slot;
  uint8_t isLoop;
  uint16_t frames;
  uint16_t frameIntervalMs;
};

enum SystemEventType {
  EV_NONE,
  EV_DMX_REC_STATIC_DONE,
  EV_DMX_REC_LOOP_DONE,
  EV_FS_LOAD_OK,
  EV_FS_LOAD_FAIL,
  EV_FS_SAVE_OK,
  EV_FS_SAVE_FAIL,
  EV_FS_SCAN_DONE,
  EV_FS_DELETE_DONE
};

struct SystemEvent {
  SystemEventType type;
  uint16_t frameCount;
  uint16_t interval;
  uint8_t mode;
  uint8_t slotMask;
};

SemaphoreHandle_t dmxMutex;
QueueHandle_t dmxCmdQueue;
QueueHandle_t fsCmdQueue;
QueueHandle_t eventQueue;

/* =========================================================================
 * LVGL DISPLAY + TOUCH DRIVERS
 * ========================================================================= */

static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 120];

lv_obj_t *active_screen = nullptr;

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area,
                   lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  M5.Display.pushImage(area->x1, area->y1, w, h, (const uint16_t *)color_p);
  lv_disp_flush_ready(disp_drv);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  if (M5.Touch.getCount() > 0) {
    auto touch = M5.Touch.getDetail();
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touch.x;
    data->point.y = touch.y;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// Forward declarations
void drawNumpad();
void drawSettingsMenu();
void drawSlotMenu(SlotMenuMode mode, uint8_t slotMask = 0xFF);
void drawRecModeMenu();
void drawMessage(String msg, lv_color_t bg_color = lv_color_black());
void drawDmxAlertScreen();

void switchScreen(lv_obj_t *new_screen) {
  if (active_screen != nullptr)
    lv_obj_del_async(active_screen);
  active_screen = new_screen;
  lv_scr_load(active_screen);
}

/* =========================================================================
 * 4. PSRAM BUFFER ALLOCATION
 * ========================================================================= */

void allocatePSRAMBuffer() {
  dmxBuffer = (uint8_t **)heap_caps_malloc(MAX_FRAMES * sizeof(uint8_t *),
                                           MALLOC_CAP_SPIRAM);
  if (!dmxBuffer) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextDatum(MC_DATUM);
    M5.Display.drawString("PSRAM ALLOC FAILED!", M5.Display.width() / 2,
                          M5.Display.height() / 2);
    while (1)
      delay(100);
  }
  for (int i = 0; i < MAX_FRAMES; i++) {
    dmxBuffer[i] =
        (uint8_t *)heap_caps_malloc(DMX_PACKET_SIZE, MALLOC_CAP_SPIRAM);
    if (!dmxBuffer[i]) {
      M5.Display.fillScreen(TFT_RED);
      M5.Display.setTextColor(TFT_WHITE);
      M5.Display.setTextDatum(MC_DATUM);
      M5.Display.drawString("FRAME BUFFER ALLOC FAILED!",
                            M5.Display.width() / 2, M5.Display.height() / 2);
      for (int j = 0; j < i; j++)
        heap_caps_free(dmxBuffer[j]);
      heap_caps_free(dmxBuffer);
      dmxBuffer = NULL;
      while (1)
        delay(100);
    }
  }
}

/* =========================================================================
 * 5. LOOP DETECTION
 * ========================================================================= */

static uint8_t activeChannelMask[DMX_PACKET_SIZE];
static uint16_t activeChannelCount = 0;
static uint16_t lastActiveChannelCount = 0;
static uint32_t profiledUpToFrame = 0;
static uint8_t chMin[DMX_PACKET_SIZE];
static uint8_t chMax[DMX_PACKET_SIZE];
static bool profileInitialized = false;
static uint16_t searchCursor = 0;
static bool searchActive = false;

static void resetLoopDetectionState() {
  profiledUpToFrame = 0;
  activeChannelCount = 0;
  lastActiveChannelCount = 0;
  profileInitialized = false;
  searchCursor = 0;
  searchActive = false;
  memset(activeChannelMask, 0, sizeof(activeChannelMask));
}

static void updateActiveChannelProfile(uint16_t totalFrames) {
  const uint8_t ACTIVE_THRESHOLD = 8;

  if (totalFrames < 2)
    return;

  if (!profileInitialized) {
    memset(chMin, 255, sizeof(chMin));
    memset(chMax, 0, sizeof(chMax));
    profileInitialized = true;
    profiledUpToFrame = 0;
  }

  for (uint16_t f = profiledUpToFrame; f < totalFrames; f++) {
    for (int c = 1; c < DMX_PACKET_SIZE; c++) {
      if (activeChannelMask[c])
        continue;
      uint8_t v = dmxBuffer[f][c];
      if (v < chMin[c])
        chMin[c] = v;
      if (v > chMax[c])
        chMax[c] = v;
      if ((uint8_t)(chMax[c] - chMin[c]) >= ACTIVE_THRESHOLD) {
        activeChannelMask[c] = 1;
        activeChannelCount++;
      }
    }
  }

  profiledUpToFrame = totalFrames;
}

static float windowSad(uint16_t startA, uint16_t startB, uint16_t period) {
  if (activeChannelCount == 0)
    return 0.0f;
  long total = 0;
  for (uint16_t f = 0; f < period; f++) {
    for (int c = 1; c < DMX_PACKET_SIZE; c++) {
      if (!activeChannelMask[c])
        continue;
      total +=
          abs((int)dmxBuffer[startA + f][c] - (int)dmxBuffer[startB + f][c]);
    }
  }
  return (float)total / ((float)activeChannelCount * (float)period);
}

static float seamSad(uint16_t period) {
  if (activeChannelCount == 0 || period < 2)
    return 0.0f;
  long total = 0;
  for (int c = 1; c < DMX_PACKET_SIZE; c++) {
    if (!activeChannelMask[c])
      continue;
    total += abs((int)dmxBuffer[period - 1][c] - (int)dmxBuffer[0][c]);
  }
  return (float)total / (float)activeChannelCount;
}

int detectLoopPeriod(uint16_t totalFrames) {
  const uint16_t MIN_PERIOD = DMX_FRAME_RATE;
  const float MATCH_THRESHOLD = 4.0f;
  const float SEAM_THRESHOLD = 8.0f;

  if (totalFrames < MIN_PERIOD * 2)
    return -1;

  updateActiveChannelProfile(totalFrames);
  if (activeChannelCount == 0)
    return -1;

  const uint16_t MAX_PERIOD = totalFrames / 2;

  if (!searchActive) {
    searchCursor = MIN_PERIOD;
    searchActive = true;
  }

  // If the channel profile changed, reset search cursor to re-evaluate
  if (activeChannelCount != lastActiveChannelCount) {
    lastActiveChannelCount = activeChannelCount;
    searchCursor = MIN_PERIOD;
  }

  const uint32_t READS_PER_SLICE = 20000;
  uint32_t readsThisSlice = 0;

  while (searchCursor <= MAX_PERIOD) {
    uint16_t P = searchCursor;
    uint32_t cost = (uint32_t)P * activeChannelCount * 2 + activeChannelCount;

    if (readsThisSlice > 0 && readsThisSlice + cost > READS_PER_SLICE) {
      return -1;
    }
    readsThisSlice += cost;
    searchCursor++;

    float sad1 = windowSad(0, P, P);
    if (sad1 >= MATCH_THRESHOLD)
      continue;

    float seam = seamSad(P);
    if (seam >= SEAM_THRESHOLD) {
      log_e("[LOOP] P=%d SAD1=%.2f seam FAIL %.2f", P, sad1, seam);
      continue;
    }

    log_e("[LOOP] CONFIRMED P=%d (%.2fs) SAD1=%.2f seam=%.2f", P,
          (float)P / DMX_FRAME_RATE, sad1, seam);
    searchActive = false;
    searchCursor = 0;
    return P;
  }

  return -1;
}

/* =========================================================================
 * 6. GUI (LVGL)
 * ========================================================================= */

static lv_obj_t *pin_label = nullptr;

void drawNumpad() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "INSERISCI PIN");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  pin_label = lv_label_create(scr);
  lv_label_set_text(pin_label, "");
  lv_obj_set_style_text_color(pin_label, lv_color_white(), 0);
  lv_obj_align(pin_label, LV_ALIGN_TOP_MID, 0, 45);

  static const char *btn_map[] = {"1", "2", "3", "\n", "4", "5", "6",  "\n",
                                  "7", "8", "9", "\n", "C", "0", "OK", ""};
  lv_obj_t *btnm = lv_btnmatrix_create(scr);
  lv_btnmatrix_set_map(btnm, btn_map);
  lv_obj_set_size(btnm, 260, 170);
  lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, 0, -5);
  lv_obj_set_style_bg_color(btnm, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(btnm, lv_color_hex(0x555555), LV_PART_ITEMS);
  lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS);

  lv_obj_add_event_cb(
      btnm,
      [](lv_event_t *e) {
        lv_obj_t *obj = lv_event_get_target(e);
        uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        if (id == LV_BTNMATRIX_BTN_NONE)
          return;
        const char *txt = lv_btnmatrix_get_btn_text(obj, id);
        if (!txt)
          return;

        if (strcmp(txt, "C") == 0) {
          inputPin = "";
        } else if (strcmp(txt, "OK") == 0) {
          if (inputPin == SECURITY_PIN) {
            inputPin = "";
            currentState = STATE_SETTINGS_MENU;
            drawSettingsMenu();
            return;
          }
          inputPin = "";
        } else {
          if (inputPin.length() < 4)
            inputPin += txt;
        }
        String masked = "";
        for (size_t i = 0; i < inputPin.length(); i++)
          masked += "*";
        lv_label_set_text(pin_label, masked.c_str());
      },
      LV_EVENT_CLICKED, NULL);

  switchScreen(scr);
}

void drawSettingsMenu() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "SETTINGS");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *btn_rec = lv_btn_create(scr);
  lv_obj_set_size(btn_rec, 240, 50);
  lv_obj_align(btn_rec, LV_ALIGN_CENTER, 0, -50);
  lv_obj_set_style_bg_color(btn_rec, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_t *lbl_rec = lv_label_create(btn_rec);
  lv_label_set_text(lbl_rec, "RECORD SCENES");
  lv_obj_center(lbl_rec);
  lv_obj_add_event_cb(
      btn_rec,
      [](lv_event_t *) {
        currentState = STATE_SLOT_SELECT_REC;
        drawMessage("...");
        FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
        xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  lv_obj_t *btn_clear = lv_btn_create(scr);
  lv_obj_set_size(btn_clear, 240, 50);
  lv_obj_align(btn_clear, LV_ALIGN_CENTER, 0, 10);
  lv_obj_set_style_bg_color(btn_clear, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_t *lbl_clear = lv_label_create(btn_clear);
  lv_label_set_text(lbl_clear, "CLEAR SCENES");
  lv_obj_center(lbl_clear);
  lv_obj_add_event_cb(
      btn_clear,
      [](lv_event_t *) {
        currentState = STATE_SLOT_SELECT_CLEAR;
        drawMessage("...");
        FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
        xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  lv_obj_t *btn_back = lv_btn_create(scr);
  lv_obj_set_size(btn_back, 240, 50);
  lv_obj_align(btn_back, LV_ALIGN_CENTER, 0, 70);
  lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_t *lbl_back = lv_label_create(btn_back);
  lv_label_set_text(lbl_back, "BACK TO PLAY");
  lv_obj_center(lbl_back);
  lv_obj_add_event_cb(
      btn_back,
      [](lv_event_t *) {
        currentState = STATE_SLOT_SELECT_PLAY;
        drawMessage("...");
        FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
        xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  switchScreen(scr);
}

void drawSlotMenu(SlotMenuMode mode, uint8_t slotMask) {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  if (mode == SLOT_MENU_PLAY) {
    lv_obj_t *status_bar = lv_obj_create(scr);
    lv_obj_set_size(status_bar, 320, 35);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_palette_darken(LV_PALETTE_GREY, 3),
                              0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    lv_obj_set_style_radius(status_bar, 0, 0);
    lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dmx_lbl = lv_label_create(status_bar);
    lv_label_set_text(dmx_lbl, "DMX IN: --");
    lv_obj_set_style_text_color(dmx_lbl, lv_color_white(), 0);
    lv_obj_align(dmx_lbl, LV_ALIGN_LEFT_MID, 10, 0);
    dmxInLabel = dmx_lbl;
    lv_obj_add_event_cb(
        dmx_lbl, [](lv_event_t *) { dmxInLabel = nullptr; }, LV_EVENT_DELETE,
        NULL);

    lv_obj_t *temp_lbl = lv_label_create(status_bar);
    lv_label_set_text(temp_lbl, "CPU: --°C");
    lv_obj_set_style_text_color(temp_lbl, lv_color_white(), 0);
    lv_obj_align(temp_lbl, LV_ALIGN_CENTER, 0, 0);
    tempLabel = temp_lbl;
    lv_obj_add_event_cb(
        temp_lbl, [](lv_event_t *) { tempLabel = nullptr; }, LV_EVENT_DELETE,
        NULL);

    lv_obj_t *btn_settings = lv_btn_create(status_bar);
    lv_obj_set_size(btn_settings, 40, 30);
    lv_obj_align(btn_settings, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_clear_flag(btn_settings, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *lbl_settings = lv_label_create(btn_settings);
    lv_label_set_text(lbl_settings, LV_SYMBOL_SETTINGS);
    lv_obj_center(lbl_settings);
    lv_obj_add_event_cb(
        btn_settings,
        [](lv_event_t *) {
          currentState = STATE_PIN_LOCK;
          drawNumpad();
        },
        LV_EVENT_CLICKED, NULL);
  }

  lv_obj_t *title = lv_label_create(scr);
  if (mode == SLOT_MENU_REC) {
    lv_label_set_text(title, "SELECT REC SLOT");
  } else if (mode == SLOT_MENU_CLEAR) {
    lv_label_set_text(title, "SELECT CLEAR SLOT");
  } else {
    lv_label_set_text(title, "SELECT PLAY SLOT");
  }
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, (mode == SLOT_MENU_PLAY) ? 45 : 10);

  lv_obj_t *grid = lv_obj_create(scr);
  lv_obj_set_size(grid, 300, 150);
  lv_obj_align(grid, LV_ALIGN_CENTER, 0, 10);
  lv_obj_set_style_bg_opa(grid, 0, 0);
  lv_obj_set_style_border_width(grid, 0, 0);
  lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  for (int i = 0; i < NUM_SCENES; i++) {
    bool exists = (slotMask >> i) & 1;

    lv_obj_t *btn = lv_btn_create(grid);
    lv_obj_set_size(btn, 60, 60);

    lv_color_t bg;
    if (mode == SLOT_MENU_REC) {
      bg = exists ? lv_palette_main(LV_PALETTE_ORANGE)
                  : lv_palette_darken(LV_PALETTE_RED, 2);
    } else if (mode == SLOT_MENU_CLEAR) {
      bg = exists ? lv_palette_main(LV_PALETTE_RED)
                  : lv_palette_main(LV_PALETTE_GREY);
    } else {
      bg = exists ? lv_palette_main(LV_PALETTE_BLUE)
                  : lv_palette_main(LV_PALETTE_GREY);
    }
    lv_obj_set_style_bg_color(btn, bg, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text_fmt(lbl, "%d", i + 1);
    lv_obj_center(lbl);

    btn->user_data = (void *)(uintptr_t)(i + 1);
    lv_obj_add_event_cb(
        btn,
        [](lv_event_t *e) {
          selectedSlot = (uintptr_t)lv_event_get_target(e)->user_data;
          if (currentState == STATE_SLOT_SELECT_REC) {
            currentState = STATE_REC_MODE_SELECT;
            drawRecModeMenu();
          } else if (currentState == STATE_SLOT_SELECT_CLEAR) {
            currentState = STATE_LOADING;
            drawMessage("CANCELLAZIONE...");
            FsCommand cmd = {FS_CMD_DELETE, selectedSlot, 0, 0, 0};
            xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
          } else {
            currentState = STATE_LOADING;
            drawMessage("CARICAMENTO...");
            FsCommand cmd = {FS_CMD_LOAD, selectedSlot, 0, 0, 0};
            xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
          }
        },
        LV_EVENT_CLICKED, NULL);
  }

  if (mode == SLOT_MENU_REC || mode == SLOT_MENU_CLEAR) {
    lv_obj_t *btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(
        btn_back,
        [](lv_event_t *) {
          currentState = STATE_SETTINGS_MENU;
          drawSettingsMenu();
        },
        LV_EVENT_CLICKED, NULL);
  }

  switchScreen(scr);
}

void drawRecModeMenu() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text_fmt(title, "SLOT %d - MODALITA", selectedSlot);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *btn_static = lv_btn_create(scr);
  lv_obj_set_size(btn_static, 120, 100);
  lv_obj_align(btn_static, LV_ALIGN_LEFT_MID, 20, 0);
  lv_obj_set_style_bg_color(btn_static, lv_palette_main(LV_PALETTE_PURPLE), 0);
  lv_obj_t *lbl_static = lv_label_create(btn_static);
  lv_label_set_text(lbl_static, "STATIC");
  lv_obj_center(lbl_static);
  lv_obj_add_event_cb(
      btn_static,
      [](lv_event_t *) {
        activeMode = 0;
        currentState = STATE_RECORDING_STATIC;
        drawMessage("RECORDING IN CORSO...", lv_palette_main(LV_PALETTE_RED));
        DmxCommand cmd = {DMX_CMD_REC_STATIC, 0, 0, 0};
        xQueueSend(dmxCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  lv_obj_t *btn_loop = lv_btn_create(scr);
  lv_obj_set_size(btn_loop, 120, 100);
  lv_obj_align(btn_loop, LV_ALIGN_RIGHT_MID, -20, 0);
  lv_obj_set_style_bg_color(btn_loop, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_t *lbl_loop = lv_label_create(btn_loop);
  lv_label_set_text(lbl_loop, "LOOP");
  lv_obj_center(lbl_loop);
  lv_obj_add_event_cb(
      btn_loop,
      [](lv_event_t *) {
        activeMode = 1;
        currentState = STATE_RECORDING_LOOP;
        drawMessage("RECORDING IN CORSO...", lv_palette_main(LV_PALETTE_RED));
        DmxCommand cmd = {DMX_CMD_REC_LOOP, 0, 0, 0};
        xQueueSend(dmxCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  switchScreen(scr);
}

void drawMessage(String msg, lv_color_t bg_color) {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, bg_color, 0);
  lv_obj_t *label = lv_label_create(scr);
  lv_label_set_text(label, msg.c_str());
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_center(label);
  switchScreen(scr);
}

void drawDmxAlertScreen() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  lv_obj_t *card = lv_obj_create(scr);
  lv_obj_set_size(card, 280, 180);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(card, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
  lv_obj_set_style_border_color(card, lv_palette_main(LV_PALETTE_AMBER), 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *icon = lv_label_create(card);
  lv_label_set_text(icon, LV_SYMBOL_WARNING " WARNING");
  lv_obj_set_style_text_color(icon, lv_palette_main(LV_PALETTE_AMBER), 0);
  lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 15);

  lv_obj_t *lbl = lv_label_create(card);
  lv_label_set_text(lbl,
                    "Please disconnect DMX\ninput device\nto allow playback");
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 15);

  lv_obj_t *btn_back = lv_btn_create(card);
  lv_obj_set_size(btn_back, 90, 35);
  lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -5);
  lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_t *lbl_back = lv_label_create(btn_back);
  lv_label_set_text(lbl_back, "CANCEL");
  lv_obj_center(lbl_back);
  lv_obj_add_event_cb(
      btn_back,
      [](lv_event_t *) {
        currentState = STATE_SLOT_SELECT_PLAY;
        drawMessage("...");
        FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
        xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
      },
      LV_EVENT_CLICKED, NULL);

  switchScreen(scr);
}

volatile bool stopPending = false;
volatile uint32_t stopTime = 0;

void drawPlayingScreen() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_palette_darken(LV_PALETTE_GREEN, 3), 0);

  lv_obj_t *label = lv_label_create(scr);
  lv_label_set_text_fmt(label, "PLAYING SLOT %d", selectedSlot);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 50);

  lv_obj_t *btn_stop = lv_btn_create(scr);
  lv_obj_set_size(btn_stop, 120, 50);
  lv_obj_align(btn_stop, LV_ALIGN_CENTER, 0, 30);
  lv_obj_set_style_bg_color(btn_stop, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_t *lbl_stop = lv_label_create(btn_stop);
  lv_label_set_text(lbl_stop, "STOP");
  lv_obj_center(lbl_stop);
  lv_obj_add_event_cb(
      btn_stop,
      [](lv_event_t *) {
        DmxCommand cmd = {DMX_CMD_STOP, 0, 0, 0};
        xQueueSend(dmxCmdQueue, &cmd, portMAX_DELAY);
        currentState = STATE_SLOT_SELECT_PLAY;
        stopPending = true;
        stopTime = millis();
      },
      LV_EVENT_CLICKED, NULL);

  switchScreen(scr);
}

/* =========================================================================
 * 7. FREERTOS TASKS
 * ========================================================================= */

// Helper: safely disable the DMX driver with retry. Returns true if it was
// enabled (and is now disabled), false if it was already disabled.
static bool dmxDriverSafeDisable() {
  if (!dmx_driver_is_enabled(DMX_PORT))
    return false;
  int retries = 0;
  while (!dmx_driver_disable(DMX_PORT)) {
    vTaskDelay(pdMS_TO_TICKS(5));
    if (++retries > 50) {
      log_e("[FS] dmx_driver_disable timeout");
      break;
    }
  }
  return true;
}

// dmxTask — pinned to Core 1.
// Owns all DMX hardware access. dmx_send_manual busy-spins here via
// ets_delay_us(), which disables the Core 1 cache temporarily. This is safe
// as long as nothing on Core 0 calls vTaskSuspend() targeting this task
// during that window. We never do that — the only cross-core serialisation
// used here is dmx_driver_disable/enable, which is ISR-safe and cache-safe.
void dmxTask(void *pvParameters) {
  DmxCmdType currentMode = DMX_CMD_IDLE;
  uint16_t playIndex = 0;
  uint8_t playMode = 0;
  uint16_t playFrameCount = 0;
  uint32_t firstFrameTime = 0;
  uint32_t lastFrameTime = 0;
  uint16_t recFrameCount = 0;

  static uint8_t taskDmxBuf[DMX_PACKET_SIZE];
  DmxCommand cmd;

  for (;;) {
    if (xQueueReceive(dmxCmdQueue, &cmd, 0) == pdTRUE) {
      switch (cmd.type) {

      case DMX_CMD_STOP:
        log_e("[DMX] STOP");
        currentMode = DMX_CMD_IDLE;
        memset(taskDmxBuf, 0, DMX_PACKET_SIZE);
        dmx_send_manual(taskDmxBuf, DMX_PACKET_SIZE);
        if (!dmx_driver_is_enabled(DMX_PORT))
          dmx_driver_enable(DMX_PORT);
        break;

      case DMX_CMD_PLAY:
        playIndex = 0;
        playMode = cmd.mode;
        playFrameCount = cmd.frameCount;
        activeFrameInterval = cmd.interval;
        currentMode = DMX_CMD_PLAY;
        log_e("[DMX] PLAY mode=%d frames=%d interval=%d", playMode,
              playFrameCount, activeFrameInterval);
        break;

      case DMX_CMD_UPDATE_INTERVAL:
        activeFrameInterval = cmd.interval;
        break;

      case DMX_CMD_REC_STATIC:
      case DMX_CMD_REC_LOOP:
        recFrameCount = 0;
        firstFrameTime = 0;
        lastFrameTime = 0;
        resetLoopDetectionState();
        currentMode = cmd.type;
        break;

      default:
        break;
      }
    }

    // --- PLAY ---
    if (currentMode == DMX_CMD_PLAY) {
      uint32_t frameStart = millis();
      if (playFrameCount > 0 && dmxBuffer != NULL) {
        if (dmx_driver_is_enabled(DMX_PORT)) {
          int r = 0;
          while (!dmx_driver_disable(DMX_PORT)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if (++r > 100)
              break;
          }
        }

        // Mutex window: pointer copy only, not the full TX
        uint8_t *framePtr = NULL;
        xSemaphoreTake(dmxMutex, portMAX_DELAY);
        framePtr = dmxBuffer[playIndex];
        xSemaphoreGive(dmxMutex);

        dmx_send_manual(framePtr, DMX_PACKET_SIZE);

        static uint32_t lastPlayLog = 0;
        if (millis() - lastPlayLog > 2000) {
          log_e("[DMX PLAY] idx=%d ch1=%d ch2=%d ch3=%d", playIndex,
                framePtr[1], framePtr[2], framePtr[3]);
          lastPlayLog = millis();
        }

        playIndex = (playMode == 1) ? (playIndex + 1) % playFrameCount : 0;
      }
      uint32_t elapsed = millis() - frameStart;
      if (elapsed < activeFrameInterval) {
        vTaskDelay(pdMS_TO_TICKS(activeFrameInterval - elapsed));
      } else {
        vTaskDelay(1);
      }

      // --- RECORD STATIC ---
    } else if (currentMode == DMX_CMD_REC_STATIC) {
      if (!dmx_driver_is_enabled(DMX_PORT)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      dmx_packet_t packet;
      if (dmx_receive(DMX_PORT, &packet, pdMS_TO_TICKS(1000))) {
        log_e("[REC_STATIC] err=%d size=%d", packet.err, packet.size);
        if (!packet.err && dmxBuffer) {
          xSemaphoreTake(dmxMutex, portMAX_DELAY);
          dmx_read(DMX_PORT, dmxBuffer[0], DMX_PACKET_SIZE);
          xSemaphoreGive(dmxMutex);
          log_e("[REC_STATIC] ch1=%d ch2=%d ch3=%d ch4=%d ch5=%d",
                dmxBuffer[0][1], dmxBuffer[0][2], dmxBuffer[0][3],
                dmxBuffer[0][4], dmxBuffer[0][5]);
          SystemEvent ev = {EV_DMX_REC_STATIC_DONE, 0, 22, 0, 0};
          xQueueSend(eventQueue, &ev, portMAX_DELAY);
          currentMode = DMX_CMD_IDLE;
        } else {
          dmx_read(DMX_PORT, taskDmxBuf, packet.size);
        }
      } else {
        log_e("[REC_STATIC] timeout");
      }

      // --- RECORD LOOP ---
    } else if (currentMode == DMX_CMD_REC_LOOP) {
      if (!dmx_driver_is_enabled(DMX_PORT)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      dmx_packet_t packet;
      bool gotPacket = dmx_receive(DMX_PORT, &packet, pdMS_TO_TICKS(50));

      taskYIELD();

      if (gotPacket && !packet.err && dmxBuffer) {
        uint32_t now = millis();
        if (recFrameCount == 0)
          firstFrameTime = now;
        lastFrameTime = now;

        if (recFrameCount < MAX_FRAMES) {
          xSemaphoreTake(dmxMutex, portMAX_DELAY);
          dmx_read(DMX_PORT, dmxBuffer[recFrameCount], DMX_PACKET_SIZE);
          xSemaphoreGive(dmxMutex);
          recFrameCount++;
        } else {
          dmx_read(DMX_PORT, taskDmxBuf, packet.size);
        }

        int period = -1;
        if (recFrameCount >= (uint16_t)(DMX_FRAME_RATE * 2)) {
          period = detectLoopPeriod(recFrameCount);
        }

        if (period > 0 || recFrameCount >= MAX_FRAMES) {
          activeFrameCount = (period > 0) ? (uint16_t)period : recFrameCount;

          if (recFrameCount > 1) {
            uint32_t avg =
                (lastFrameTime - firstFrameTime) / (recFrameCount - 1);
            if (avg < 22)
              avg = 22;
            if (avg > 100)
              avg = 100;
            activeFrameInterval = (uint16_t)avg;
          } else {
            activeFrameInterval = 22;
          }

          resetLoopDetectionState();

          SystemEvent ev = {EV_DMX_REC_LOOP_DONE, activeFrameCount,
                            activeFrameInterval, 1, 0};
          xQueueSend(eventQueue, &ev, portMAX_DELAY);
          currentMode = DMX_CMD_IDLE;
        }
      } else if (gotPacket) {
        dmx_read(DMX_PORT, taskDmxBuf, packet.size);
      }

      // --- IDLE ---
    } else {
      if (!dmx_driver_is_enabled(DMX_PORT)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      dmx_packet_t packet;
      if (dmx_receive(DMX_PORT, &packet, pdMS_TO_TICKS(50))) {
        if (!packet.err)
          lastDmxRxTime = millis();
        dmx_read(DMX_PORT, taskDmxBuf, packet.size);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

// fsTask — pinned to Core 0, priority 3.
// All flash I/O lives here. Uses dmx_driver_disable/enable for cross-core
// cache safety — this is the ONLY correct primitive on ESP32-S3/IDF5.
// vTaskSuspend() targeting a Core 1 task is NEVER used because it races with
// ets_delay_us() cache-disable windows inside dmx_send_manual, causing
// EXCCAUSE 7 panics.
void fsTask(void *pvParameters) {
  const size_t BUF_SIZE = 4096;
  // Internal SRAM staging buffer: faster than PSRAM for flash I/O path
  uint8_t *rwBuffer = (uint8_t *)malloc(BUF_SIZE);
  if (!rwBuffer) {
    rwBuffer = (uint8_t *)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM);
    log_e("[FS] rwBuffer fell back to PSRAM");
  }

  FsCommand cmd;
  for (;;) {
    if (xQueueReceive(fsCmdQueue, &cmd, portMAX_DELAY) != pdTRUE)
      continue;

    char filename[32];
    sprintf(filename, "/scene_%d.bin", cmd.slot);

    // ---- SCAN SLOTS ----
    // LittleFS.exists() reads flash. Disable the DMX driver first so that
    // Core 1 is not touching the flash/cache bus concurrently.
    if (cmd.type == FS_CMD_SCAN_SLOTS) {
      bool wasEnabled = dmxDriverSafeDisable();

      uint8_t mask = 0;
      char fn[32];
      for (int i = 0; i < NUM_SCENES; i++) {
        sprintf(fn, "/scene_%d.bin", i + 1);
        if (LittleFS.exists(fn))
          mask |= (1 << i);
        vTaskDelay(pdMS_TO_TICKS(2));
      }

      if (wasEnabled)
        dmx_driver_enable(DMX_PORT);

      SystemEvent ev = {EV_FS_SCAN_DONE, 0, 0, 0, mask};
      xQueueSend(eventQueue, &ev, portMAX_DELAY);
      continue;
    }

    // ---- SAVE / LOAD ----
    bool dmxWasEnabled = dmxDriverSafeDisable();

    if (cmd.type == FS_CMD_SAVE) {
      char tmpName[40];
      sprintf(tmpName, "%s.tmp", filename);
      File file = LittleFS.open(tmpName, FILE_WRITE);
      if (!file || !dmxBuffer) {
        SystemEvent ev = {EV_FS_SAVE_FAIL, 0, 0, 0, 0};
        xQueueSend(eventQueue, &ev, portMAX_DELAY);
        if (file)
          file.close();
        goto cleanup_fs;
      }

      {
        SceneFileHeader hdr = {cmd.isLoop, cmd.frames, cmd.frameIntervalMs};
        file.write((uint8_t *)&hdr, sizeof(hdr));

        if (rwBuffer) {
          size_t bufPos = 0;
          for (int i = 0; i < cmd.frames; i++) {
            if (bufPos + DMX_PACKET_SIZE > BUF_SIZE) {
              file.write(rwBuffer, bufPos);
              bufPos = 0;
              vTaskDelay(pdMS_TO_TICKS(5));
            }
            memcpy(rwBuffer + bufPos, dmxBuffer[i], DMX_PACKET_SIZE);
            bufPos += DMX_PACKET_SIZE;
          }
          if (bufPos > 0)
            file.write(rwBuffer, bufPos);
        } else {
          for (int i = 0; i < cmd.frames; i++) {
            file.write(dmxBuffer[i], DMX_PACKET_SIZE);
            vTaskDelay(pdMS_TO_TICKS(5));
          }
        }
      }
      file.close();
      LittleFS.rename(String(tmpName), String(filename));
      {
        SystemEvent ev = {EV_FS_SAVE_OK, 0, 0, 0, 0};
        xQueueSend(eventQueue, &ev, portMAX_DELAY);
      }

    } else if (cmd.type == FS_CMD_LOAD) {
      File file = LittleFS.open(filename, FILE_READ);
      if (!file || !dmxBuffer) {
        SystemEvent ev = {EV_FS_LOAD_FAIL, 0, 0, 0, 0};
        xQueueSend(eventQueue, &ev, portMAX_DELAY);
        if (file)
          file.close();
        goto cleanup_fs;
      }

      SceneFileHeader hdr;
      if (file.read((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) {
        SystemEvent ev = {EV_FS_LOAD_FAIL, 0, 0, 0, 0};
        xQueueSend(eventQueue, &ev, portMAX_DELAY);
        file.close();
        goto cleanup_fs;
      }

      uint16_t loadFrames = min((uint16_t)hdr.frames, (uint16_t)MAX_FRAMES);
      uint16_t loadInterval =
          (hdr.frameIntervalMs >= 22 && hdr.frameIntervalMs <= 100)
              ? hdr.frameIntervalMs
              : 22;

      if (rwBuffer) {
        size_t bytesLeft = (size_t)loadFrames * DMX_PACKET_SIZE;
        size_t frameIdx = 0;
        size_t frameOff = 0;
        while (bytesLeft > 0 && frameIdx < loadFrames) {
          size_t toRead = min(bytesLeft, BUF_SIZE);
          size_t didRead = file.read(rwBuffer, toRead);
          if (didRead == 0)
            break;
          size_t pos = 0;
          while (pos < didRead && frameIdx < loadFrames) {
            size_t chunk =
                min((size_t)(DMX_PACKET_SIZE - frameOff), didRead - pos);
            memcpy(dmxBuffer[frameIdx] + frameOff, rwBuffer + pos, chunk);
            frameOff += chunk;
            pos += chunk;
            if (frameOff >= DMX_PACKET_SIZE) {
              frameOff = 0;
              frameIdx++;
            }
          }
          bytesLeft -= didRead;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        loadFrames = (uint16_t)frameIdx;
      } else {
        for (uint16_t i = 0; i < loadFrames; i++) {
          if (file.read(dmxBuffer[i], DMX_PACKET_SIZE) < DMX_PACKET_SIZE) {
            loadFrames = i;
            break;
          }
          vTaskDelay(pdMS_TO_TICKS(2));
        }
      }
      file.close();

      log_e("[FS_LOAD] slot=%d frames=%d ch1=%d ch2=%d ch3=%d", cmd.slot,
            loadFrames, dmxBuffer[0][1], dmxBuffer[0][2], dmxBuffer[0][3]);

      SystemEvent ev = {EV_FS_LOAD_OK, loadFrames, loadInterval, hdr.isLoop, 0};
      xQueueSend(eventQueue, &ev, portMAX_DELAY);
    } else if (cmd.type == FS_CMD_DELETE) {
      LittleFS.remove(filename);
      log_e("[FS_DELETE] slot=%d filename=%s", cmd.slot, filename);
      SystemEvent ev = {EV_FS_DELETE_DONE, 0, 0, 0, 0};
      xQueueSend(eventQueue, &ev, portMAX_DELAY);
    }

  cleanup_fs:
    if (dmxWasEnabled)
      dmx_driver_enable(DMX_PORT);
  }
}

/* =========================================================================
 * 8. SETUP
 * ========================================================================= */

void setup() {
  dmxMutex = xSemaphoreCreateMutex();

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(128);

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.drawString("Avvio sistema in corso...", M5.Display.width() / 2,
                        M5.Display.height() / 2 - 20);
  M5.Display.drawString("Attendi (potrebbe richiedere 20-30s)",
                        M5.Display.width() / 2, M5.Display.height() / 2 + 20);

  allocatePSRAMBuffer();

  if (!LittleFS.begin(true))
    Serial.println("LITTLEFS MOUNT FAILED!");

  dmx_config_t dmxConfig = DMX_CONFIG_DEFAULT;
  dmx_driver_install(DMX_PORT, &dmxConfig, DMX_INTR_FLAGS_DEFAULT);
  dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 120);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  lv_theme_t *theme = lv_theme_default_init(
      NULL, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
      true, LV_FONT_DEFAULT);
  lv_disp_set_theme(NULL, theme);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  dmxCmdQueue = xQueueCreate(5, sizeof(DmxCommand));
  fsCmdQueue = xQueueCreate(5, sizeof(FsCommand));
  eventQueue = xQueueCreate(10, sizeof(SystemEvent));

  // dmxTask → Core 1: ets_delay_us busy-spins here, isolated from UI core.
  // fsTask  → Core 0: flash I/O on same core as loop()/LVGL; yields via
  //                   vTaskDelay so UI remains responsive between chunks.
  xTaskCreatePinnedToCore(dmxTask, "DmxTask", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(fsTask, "FsTask", 8192, NULL, 3, NULL, 0);

  currentState = STATE_SLOT_SELECT_PLAY;
  drawMessage("...");
  FsCommand scanCmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
  xQueueSend(fsCmdQueue, &scanCmd, portMAX_DELAY);
}

/* =========================================================================
 * 9. MAIN LOOP (Core 0, priority 1)
 * ========================================================================= */

void loop() {
  M5.update();
  lv_timer_handler();
  delay(5);

  if (currentState == STATE_DMX_INPUT_ALERT) {
    if (millis() - lastDmxRxTime >= 1000) {
      currentState = STATE_PLAYING_SCENE;
      {
        DmxCommand dmxCmd = {DMX_CMD_PLAY, activeMode,
                             (uint16_t)activeFrameCount,
                             (uint16_t)activeFrameInterval};
        xQueueSend(dmxCmdQueue, &dmxCmd, portMAX_DELAY);
      }
      drawPlayingScreen();
    }
  }

  if (currentState == STATE_SLOT_SELECT_PLAY && stopPending) {
    if (millis() - stopTime > 200) {
      stopPending = false;
      drawMessage("...");
      FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
      xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
    }
  }

  static uint32_t lastUiUpdate = 0;
  if (millis() - lastUiUpdate > 100) {
    if (dmxInLabel && lv_obj_is_valid(dmxInLabel)) {
      bool active = (millis() - lastDmxRxTime < 1000);
      lv_label_set_text(dmxInLabel, active ? "DMX IN: ON" : "DMX IN: OFF");
      lv_obj_set_style_text_color(
          dmxInLabel, active ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000),
          0);
    }
    lastUiUpdate = millis();
  }

  static uint32_t lastTempUpdate = 0;
  static float filteredTemp = 0;
  if (millis() - lastTempUpdate > 1000) {
    if (tempLabel && lv_obj_is_valid(tempLabel)) {
      float raw = temperatureRead();
      if (filteredTemp == 0)
        filteredTemp = raw;
      filteredTemp = filteredTemp * 0.8f + raw * 0.2f;
      lv_label_set_text_fmt(tempLabel, "CPU: %d°C", (int)(filteredTemp + 0.5f));
      lv_color_t col = (filteredTemp > 70.0f) ? lv_palette_main(LV_PALETTE_RED)
                       : (filteredTemp > 55.0f)
                           ? lv_palette_main(LV_PALETTE_ORANGE)
                           : lv_color_white();
      lv_obj_set_style_text_color(tempLabel, col, 0);
    }
    lastTempUpdate = millis();
  }

  static uint32_t messageTimeout = 0;
  static SystemState nextStateAfterMessage = STATE_SLOT_SELECT_PLAY;
  if (messageTimeout > 0 && millis() > messageTimeout) {
    messageTimeout = 0;
    if (currentState == STATE_SAVING || currentState == STATE_LOADING) {
      currentState = nextStateAfterMessage;
      drawMessage("...");
      FsCommand cmd = {FS_CMD_SCAN_SLOTS, 0, 0, 0, 0};
      xQueueSend(fsCmdQueue, &cmd, portMAX_DELAY);
    }
  }

  SystemEvent ev;
  if (xQueueReceive(eventQueue, &ev, 0) != pdTRUE)
    return;

  switch (ev.type) {

  case EV_FS_SCAN_DONE: {
    SlotMenuMode mode = SLOT_MENU_PLAY;
    if (currentState == STATE_SLOT_SELECT_REC) {
      mode = SLOT_MENU_REC;
    } else if (currentState == STATE_SLOT_SELECT_CLEAR) {
      mode = SLOT_MENU_CLEAR;
    }
    drawSlotMenu(mode, ev.slotMask);
    break;
  }

  case EV_DMX_REC_STATIC_DONE: {
    FsCommand fsCmd = {FS_CMD_SAVE, selectedSlot, 0, 1, ev.interval};
    xQueueSend(fsCmdQueue, &fsCmd, portMAX_DELAY);
    currentState = STATE_SAVING;
    drawMessage("SALVATAGGIO...");
    break;
  }

  case EV_DMX_REC_LOOP_DONE: {
    FsCommand fsCmd = {FS_CMD_SAVE, selectedSlot, 1, ev.frameCount,
                       ev.interval};
    xQueueSend(fsCmdQueue, &fsCmd, portMAX_DELAY);
    currentState = STATE_SAVING;
    drawMessage("SALVATAGGIO...");
    break;
  }

  case EV_FS_SAVE_OK:
    drawMessage("SALVATO!");
    messageTimeout = millis() + 1000;
    nextStateAfterMessage = STATE_SETTINGS_MENU;
    break;

  case EV_FS_SAVE_FAIL:
    drawMessage("ERRORE SALVATAGGIO", lv_palette_main(LV_PALETTE_RED));
    messageTimeout = millis() + 1000;
    nextStateAfterMessage = STATE_SETTINGS_MENU;
    break;

  case EV_FS_LOAD_OK:
    activeMode = ev.mode;
    activeFrameCount = ev.frameCount;
    activeFrameInterval = ev.interval;
    if (millis() - lastDmxRxTime < 1000) {
      currentState = STATE_DMX_INPUT_ALERT;
      drawDmxAlertScreen();
    } else {
      currentState = STATE_PLAYING_SCENE;
      {
        DmxCommand dmxCmd = {DMX_CMD_PLAY, activeMode,
                             (uint16_t)activeFrameCount,
                             (uint16_t)activeFrameInterval};
        xQueueSend(dmxCmdQueue, &dmxCmd, portMAX_DELAY);
      }
      drawPlayingScreen();
    }
    break;

  case EV_FS_LOAD_FAIL:
    drawMessage("ERRORE CARICAMENTO", lv_palette_main(LV_PALETTE_RED));
    messageTimeout = millis() + 1000;
    nextStateAfterMessage = STATE_SLOT_SELECT_PLAY;
    break;

  case EV_FS_DELETE_DONE:
    drawMessage("CANCELLATO!", lv_palette_main(LV_PALETTE_RED));
    messageTimeout = millis() + 1000;
    nextStateAfterMessage = STATE_SLOT_SELECT_CLEAR;
    break;

  default:
    break;
  }
}