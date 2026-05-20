# M5Stack CoreS3 DMX Standalone Recorder & Player

A robust, multi-core DMX512 recording and playback controller built for the **M5Stack CoreS3 SE** with the **Base DMX** module. 

This firmware transforms the M5Stack into a standalone DMX tool capable of capturing live DMX streams, detecting loops, and playing them back without needing a lighting console. It is ideal for compact AV service setups or for dropping into long-term architectural and art exhibition installations where a dedicated console is overkill.

## 🚀 Key Features

* **Dual-Mode Recording:**
    * **Static Mode:** Captures a single snapshot of the current DMX state.
    * **Loop Mode:** Records up to 20 seconds of dynamic DMX data (at 44Hz) and features an auto-detection algorithm to find seamless loop periods based on active channel changes.
* **8 Scene Slots:** Save and recall up to 8 different scenes securely to the internal LittleFS flash memory.
* **Touch UI (LVGL):** Fully interactive touchscreen interface for slot selection, recording, clearing, and system monitoring.
* **Security PIN Lock:** Settings and recording menus are locked behind a PIN code (`0000` by default) to prevent tampering during live events or exhibitions.
* **Hardware CPU Temp & DMX Monitoring:** Real-time UI updates indicating active DMX input and internal CPU temperatures.

## 🛠 Hardware Requirements

* **Controller:** M5Stack CoreS3 SE
* **Expansion:** M5Stack Base DMX (or compatible RS485/DMX transceiver wired to the M5Stack bus)
* **Pin Mapping:**
    * `DMX_TX_PIN`: 7
    * `DMX_RX_PIN`: 10
    * `DMX_EN_PIN`: 6

## ⚙️ Software Architecture & Technical Highlights

This project relies on **FreeRTOS** to ensure the LVGL graphical interface does not interrupt the timing requirements of the DMX512 protocol.

### 1. Dual-Core Isolation
* **Core 1 (`dmxTask`):** Strictly handles DMX transmission and reception. It bypasses standard timer interrupts (which can cause panics on ESP32-S3 under IDF5) and utilizes a custom manual UART bit-banging method (`dmx_send_manual()`) for ultra-stable transmission.
* **Core 0 (`fsTask` & `loop`):** Manages the LVGL touchscreen UI, LittleFS file operations, and system events.

### 2. Cache-Safe Flash Operations
Writing to flash memory (LittleFS) temporarily disables the CPU cache. To prevent cache panics, the firmware strictly serializes flash writes and DMX operations using safe `dmx_driver_disable()` / `dmx_driver_enable()` wrappers. Core 0 will never preempt or suspend Core 1 while it is executing a busy-spin delay.

### 3. PSRAM Frame Buffering
DMX loop recording requires significant memory (up to 880 frames at 513 bytes each). At boot, the system dynamically allocates a dedicated frame buffer in the ESP32-S3's SPIRAM (PSRAM) to prevent heap fragmentation and out-of-memory crashes.

## 📦 Dependencies

Ensure the following libraries are installed in your `platformio.ini` or Arduino IDE:

* `M5Unified` (M5Stack core library)
* `lvgl` (Light and Versatile Graphics Library)
* `esp_dmx` (Someweis ESP-DMX library)
* Built-in ESP32 libraries: `LittleFS`, `SPI`, `Arduino`

## 🕹️ Usage Flow

1.  **Boot & Play:** The device boots into the **Play Menu**. If scenes are saved, select a slot (blue blocks) to immediately start DMX playback.
2.  **Access Settings:** Tap the gear icon ⚙️ in the top right.
3.  **Enter PIN:** Enter the security PIN (`0000`).
4.  **Settings Menu:** Choose to **Record** or **Clear** scenes.
5.  **Recording:**
    * Ensure a DMX controller is plugged into the DMX IN port.
    * Select an empty slot.
    * Choose **STATIC** (single frame) or **LOOP** (dynamic animation).
    * *Note for Loops:* The device will profile the incoming data, look for a repeating pattern (SAD matching), calculate the frame interval, and save it to flash.
6.  **Playback Constraints:** The UI will actively prevent you from playing a scene if an active DMX Input is still detected, to avoid signal collision on the physical DMX line.

## 📝 Configuration

To change the default security PIN or adjust recording limits, modify the `CONSTANTS` section at the top of `main.cpp`:

```cpp
#define SECURITY_PIN "0000"
#define NUM_SCENES 8
#define MAX_RECORD_SEC 20
