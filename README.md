# LimeSDR GNSS Cockpit

LimeSDR GNSS Cockpit is a comprehensive graphical application built for macOS using Qt6/C++ to interface with a LimeSDR-USB and `gnss-sdr`. It provides real-time monitoring, diagnostic tools, and automated parameter tuning to help achieve and maintain GNSS/GPS signal lock, even in environments with weak signals or high phase noise.

## Features

- **Live GNSS Visualization:** 
  - Real-time PVT (Position, Velocity, Time) monitoring via NMEA parsing.
  - Graphical Skyplot showing satellite constellations and their C/N0 (SNR) values.
  - Dynamic Channel tracking status (Searching, Tracking, Locked).
- **Advanced SDR Controls:**
  - Direct control over Sample Rate and RF Gain.
  - Toggle 3.3V Bias-Tee for powering active antennas directly from the LimeSDR.
  - Adaptive Gain Tuning (Hill-Climbing algorithm) to automatically maximize satellite C/N0.
  - Auto-Tune Tracking Loops to dynamically widen PLL/DLL bandwidths if the receiver frequently loses lock due to oscillator phase noise.
- **Diagnostic Tools:**
  - **Antenna Noise Floor Test:** An interactive wizard that measures the thermal noise floor drop when unplugging the antenna, definitively proving whether the active antenna LNA is functioning and drawing power from the Bias-Tee.
  - **RF Interference Scanner:** A fast spectrum scanner that hops frequencies and plots a Max-Hold waterfall to detect local interference (like cellular or 5G bands) that might be overwhelming the SDR frontend.
- **Headless Auto-Tuning:**
  - A dedicated `--headless-tune` CLI mode that iterates through a massive grid of configurations (Gain, PLL BW, DLL BW) to find the optimal parameters for your specific hardware and environment to achieve a stable PVT fix.
- **Data Recording:**
  - Raw complex-float IQ sample recording directly to `.bin` for offline post-processing.

## Architecture

- `MainWindow`: Handles the primary GUI, Skyplot visualization, and manages the `gnss-sdr` subprocess.
- `SdrStreamer`: A dedicated `QThread` that uses the SoapySDR API to read raw baseband samples from the LimeSDR and write them to a named FIFO pipe for `gnss-sdr` to consume.
- `SdrScanner`: Rapidly sweeps frequencies using SoapySDR to build an RF spectrum profile for interference detection.
- `HeadlessTuner`: Operates when launched via `--headless-tune`, managing an unattended parameter sweep and dynamically generating `gnss-sdr.conf` files.
- `NmeaParser`: Parses GNGGA and GPGSV sentences emitted by `gnss-sdr` to update the GUI.
- `AntennaTestDialog`: Manages the step-by-step Bias-Tee LNA noise floor test.

## Prerequisites

- **macOS** (Apple Silicon or Intel)
- **Qt 6** (`brew install qt`)
- **SoapySDR** and LimeSDR drivers (`brew install soapysdr limesuite`)
- **gnss-sdr** (Compiled and available in `$PATH`)
- A **LimeSDR-USB** (or other SoapySDR compatible frontend) and a GNSS active antenna.

## Building and Running

The project uses Qt's `qmake` build system.

1. Generate the Makefile:
   ```bash
   qmake LimeSrdCockpit.pro
   ```
2. Build the application:
   ```bash
   make
   ```
3. Run the GUI:
   ```bash
   ./LimeSrdCockpit.app/Contents/MacOS/LimeSrdCockpit
   ```

### Headless Parameter Tuning

To run the automated parameter grid search (useful if you are having trouble getting a lock with default settings):

```bash
./LimeSrdCockpit.app/Contents/MacOS/LimeSrdCockpit --headless-tune
```

This will run in the terminal without opening the GUI, sweeping through 125 combinations of RF Gain, PLL bandwidth, and DLL bandwidth. Results are logged to the console, and it will automatically stop if a valid 3D Fix is acquired.

## Troubleshooting

- **"Failed to open. Device is busy."**: Another instance of the cockpit or `gnss-sdr` is still running in the background. Run `killall -9 LimeSrdCockpit gnss-sdr` to free the USB device.
- **No Satellites Found**: Ensure the Bias-Tee is enabled if using an active antenna. You can verify this using the "Test Antenna (Noise Floor)" button in the GUI.
- **Losing Lock Frequently**: LimeSDR-USBs are known for high phase noise. Enable the "Auto-Tune Tracking Loops" feature in the GUI to allow the software to widen the tracking bandwidths dynamically.
