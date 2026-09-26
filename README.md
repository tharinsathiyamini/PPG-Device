# BM2210 Assignment 3 - Reflectance PPG Wearable (Wokwi + VS Code)

## What each file does

```
ppg-project/
├── diagram.json              Wokwi wiring: ESP32 + OLED + AFE chip + USER PPG chip
├── wokwi.toml                Tells the Wokwi VS Code extension which firmware
│                             and which compiled custom-chip .wasm files to load
├── platformio.ini            ESP32 build config (Arduino framework + libraries)
├── src/
│   └── main.cpp              Task 3: ESP32 firmware (SPI master, HR/SpO2 DSP,
│                             OLED display, WiFi + MQTT alarm to HiveMQ)
└── chips/
    ├── user_ppg/
    │   ├── chip.c             Task 1: USER PPG "tissue" model (custom Wokwi chip)
    │   └── chip.json          Its pins + the HR/SpO2/PI sliders
    └── afe/
        ├── chip.c             Task 2: AFE - LED sequencer, dark subtraction,
        │                      16-bit "ADC", SPI register-map slave
        └── chip.json          Its pins (no sliders needed)
```

Data flow: **USER PPG chip → (analog PD_OUT) → AFE chip → (SPI) → ESP32 → OLED + MQTT/HiveMQ**

## One-time setup

1. Install **VS Code**, the **Wokwi extension**, and the **PlatformIO extension**.
2. Get a free Wokwi VS Code license (needed for custom chips) at https://wokwi.com/vscode.
3. Install **Node.js** (needed by `wokwi-cli`), then install the CLI once:
   ```
   npm install -g wokwi-cli
   ```
   `wokwi-cli` is the easiest way to compile a custom chip's C code to the
   WebAssembly binary Wokwi actually runs - it downloads the small
   clang/WASI toolchain for you the first time you use it, so you don't need
   to install a C cross-compiler by hand.

## Build steps

1. Copy this whole `ppg-project` folder into your own workspace and open it in VS Code.
2. Compile the two custom chips (run from the `ppg-project` root):
   ```
   wokwi-cli chip compile chips/user_ppg/chip.c -o chips/user_ppg/dist/chip.wasm
   wokwi-cli chip compile chips/afe/chip.c -o chips/afe/dist/chip.wasm
   ```
   Each command produces a `dist/chip.wasm` next to the source - that's what
   `wokwi.toml` points to. Re-run the relevant command every time you edit a
   `chip.c` file.
3. Build the ESP32 firmware with PlatformIO (VS Code PlatformIO sidebar →
   "Build", or `pio run` in the terminal). This produces
   `.pio/build/esp32dev/firmware.bin`, which `wokwi.toml` already points to.
4. Press **F1 → "Wokwi: Start Simulator"**.
5. In the diagram view, drag the **HR**, **SpO2**, and **PI** sliders on the
   `userppg` chip and confirm: the OLED pulse trace speeds up/slows down
   with HR, and the reported SpO2 number tracks the slider.
6. Open the HiveMQ public WebSocket client (http://www.hivemq.com/demos/websocket-client/),
   connect to `broker.hivemq.com`, subscribe to `bm2210/ppg-demo/alarm`
   (or whatever topic you set in `main.cpp`), then push the SpO2 slider
   below 92% (or HR outside 50-120 bpm) in the simulation and screenshot
   the alarm message arriving with its timestamp

