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
   the alarm message arriving with its timestamp - that's deliverable (3)
   in the assignment brief.

## If something doesn't wire up

- Click each part in the Wokwi diagram editor and check the exact pin
  label text shown on hover - board pinout label strings occasionally
  differ by a character between Wokwi board versions, so double-check
  `diagram.json`'s connections against what you see if a wire shows red/unconnected.
- Use the Wokwi **Logic Analyzer** on the CS/SCK/MOSI/MISO lines if the
  ESP32 isn't reading sensible register values - this lets you literally
  see the address byte and reply bytes crossing the SPI bus.
- The `chips console` panel (below the diagram) shows every `printf()`
  from your two custom chips - useful for confirming the AFE's phase
  sequencer and DRDY logic are actually toggling.

## Tuning notes (be ready to adjust these for your report)

- `DC_ALPHA` and the peak-detector threshold in `main.cpp` were chosen to
  be reasonable starting points, not verified against your exact noise/PI
  settings - watch the Serial Plotter and tune them so the dicrotic notch
  doesn't get counted as a second beat and the HR reading is stable.
- The AFE's `LED_CURRENT`/`GAIN` registers are implemented as real,
  readable/writable SPI registers (satisfying the interface requirement),
  but in this simplified simulation they don't yet feed back into the
  USER PPG chip's optical model. State this as a scope limitation in your
  report (this is exactly the kind of stated limitation the assignment
  hints ask for, like the SpO2≈110−25R calibration limitation).

## Report checklist (from the assignment brief)

1. **Block diagram** - you can adapt the one in this README's data-flow
   line, or redraw Figure 1 with your actual pin names.
2. **Timing sketch** - draw four back-to-back 1 ms windows: RED‑lit, DARK,
   IR‑lit, DARK (this matches `PHASE_US` and the phase state machine in
   `chips/afe/chip.c`), with the ADC sample instant marked at the end of
   each window.
3. **HiveMQ screenshot** - from step 6 above, showing the JSON alarm
   payload and a real date/time (via NTP - see `isoTimestamp()`).
4. **Threshold justification** - look up adult resting HR ranges and a
   clinical SpO2 alarm limit for a room-air patient (e.g. an early-warning
   score reference such as NEWS2, or a national oxygen-therapy guideline)
   and cite it; the defaults used here (50/120 bpm, 92% SpO2) are a
   reasonable starting point but you should confirm and cite your own.
5. **Source code appendix** - `chips/user_ppg/chip.c`, `chips/afe/chip.c`,
   and `src/main.cpp`.

## Bonus option (b) - motion artifact rejection

If you go for the accelerometer bonus: add an `mpu6050`-type part (Wokwi
has a built-in one) wired to the ESP32's I2C bus, add a "motion" slider or
periodic disturbance to `chips/user_ppg/chip.c`'s `PD_OUT` (e.g. add a
low-frequency, larger-amplitude term when a `MOTION` input pin is driven
high by the ESP32 reading the accelerometer), then in `main.cpp` gate the
alarm logic so a `MOTION` flag suppresses new alarms while movement is
detected. Capture before/after plots (Serial Plotter or logged to CSV) of
the SpO2 estimate with and without the rejection logic enabled.
