# Software Architecture & Specification
**Project:** LimeSDR GNSS Cockpit
**Version:** 1.1

## 1. Executive Summary

The **LimeSDR GNSS Cockpit** is a comprehensive macOS desktop application built with Qt6/C++. It serves as an orchestrator and graphical interface bridging low-cost SDR hardware (LimeSDR-USB) and the powerful `gnss-sdr` processing engine. It abstracts the complexity of RF signal acquisition, digital signal processing (DSP) tuning, and geospatial data visualization into an accessible suite of tools.

---

## 2. High-Level Architecture (System Context)

The High-Level Architecture describes how the Cockpit fits into the physical world, interacting with both the user and the external hardware/software dependencies.

```mermaid
graph TD
    User([End User])
    
    subgraph Host [macOS Host System]
        Cockpit["LimeSDR GNSS Cockpit <br> (Qt6 / C++)"]
        GNSS_SDR["gnss-sdr <br> (Standalone DSP Engine)"]
        Files[(File System)]
    end
    
    subgraph Hardware [Physical Hardware]
        LimeSDR["LimeSDR-USB <br> (SDR Hardware)"]
        Antenna((Active GNSS Antenna))
    end
    
    User <-->|GUI / CLI| Cockpit
    Cockpit <-->|libusb / SoapySDR API| LimeSDR
    LimeSDR <-->|Bias-Tee Power / RF In| Antenna
    
    Cockpit -->|IQ Samples - Named Pipe| GNSS_SDR
    Cockpit -->|Generates Config| Files
    GNSS_SDR -->|Reads Config| Files
    GNSS_SDR -->|NMEA 0183 & Logs| Files
    Cockpit <-->|Tails NMEA & Logs| Files
```

### Key Interactions:
*   **User ↔ Cockpit:** The user provides inputs (Gain, Bandwidth, Tuning goals) and receives visual feedback (Skyplots, SNR).
*   **Cockpit ↔ LimeSDR:** The application configures the hardware (Center frequency, sample rate, digital gain) and pulls raw baseband IQ samples over USB 3.0.
*   **Cockpit ↔ gnss-sdr:** The Cockpit acts as a master process, dynamically generating configuration files, spawning the `gnss-sdr` subprocess, and feeding it raw data via a POSIX Named Pipe (FIFO).
*   **gnss-sdr ↔ Cockpit (Telemetry):** `gnss-sdr` writes its PVT (Position, Velocity, Time) solutions to a local `.nmea` file, which the Cockpit tails and parses in real-time.

---

## 3. Software Architecture (Process & IPC Model)

The software architecture relies heavily on multi-threading to ensure the UI remains responsive while processing high-bandwidth data (16 MB/s for 2 MSPS CF32). It also utilizes Inter-Process Communication (IPC) to decouple the heavy correlation math of `gnss-sdr` from the GUI.

```mermaid
flowchart LR
    subgraph P1 ["Process 1: LimeSrdCockpit"]
        UI["Main GUI Thread"]
        NMEA["NMEA Parser"]
        
        subgraph WThreads ["Worker Threads"]
            SdrStreamer[["SdrStreamer Thread"]]
            SdrScanner[["SdrScanner Thread"]]
        end
    end

    subgraph P2 ["Process 2: gnss-sdr"]
        DSP["GNSS Correlation & Tracking"]
    end
    
    subgraph IPC ["IPC / File IO"]
        FIFO[(POSIX FIFO Pipe)]
        LOG[(gnss-sdr.log)]
        NMEA_FILE[(PVT Output .nmea)]
    end

    UI -->|Start/Stop/Config| SdrStreamer
    UI -->|Start/Stop| SdrScanner
    
    SdrStreamer -->|Write CF32 Samples| FIFO
    FIFO -->|Read CF32 Samples| DSP
    
    DSP -->|Write| LOG
    DSP -->|Write| NMEA_FILE
    
    NMEA_FILE -->|Tail/Read| NMEA
    LOG -->|Tail/Read| UI
    NMEA -->|Update UI| UI
```

### Concurrency Model:
1.  **Main Thread:** Handles all Qt event loops, widget painting, user interaction, and QTimer-based file tailing.
2.  **SdrStreamer Thread:** A dedicated `QThread` executing a blocking `readStream` loop against the SoapySDR API. It performs lightweight math (calculating overall RF power in dBFS) and pushes samples into the FIFO.
3.  **SdrScanner Thread:** A dedicated `QThread` used exclusively for the diagnostic scanner. It rapidly retunes the SDR frequency and pulls small buffers to construct an FFT-averaged spectrum.

---

## 4. Low-Level Architecture (Class Design)

The low-level design maps out the specific C++ classes and their relationships within the Qt framework.

```mermaid
classDiagram
    class QMainWindow {
        <<Qt>>
    }
    class QThread {
        <<Qt>>
    }
    
    QMainWindow <|-- MainWindow
    QThread <|-- SdrStreamer
    QThread <|-- SdrScanner
    
    MainWindow "1" *-- "1" SdrStreamer : Aggregates
    MainWindow "1" *-- "1" SdrScanner : Aggregates
    MainWindow "1" *-- "1" NmeaParser : Aggregates
    MainWindow "1" *-- "1" HeadlessTuner : Owns
    
    class MainWindow {
        -QProcess* m_gnssProcess
        -QTimer* m_updateTimer
        -QMap<int, ChannelState> m_channels
        +onStartClicked()
        +onAutotuneClicked()
        -writeGnssSdrConfig()
        -readNmeaFile()
    }
    
    class SdrStreamer {
        -bool m_running
        -double m_gain
        -double m_sampleRate
        +startStreaming(fifoPath, ...)
        +stopStreaming()
        #run()
        <<signal>> powerMeasured(powerDb)
    }
    
    class HeadlessTuner {
        -QList<TuneConfig> m_grid
        -int m_currentIndex
        +startSweep()
        -runNextConfig()
        -evaluateSuccess()
    }
    
    class NmeaParser {
        -double m_lat
        -double m_lon
        -QMap<int, SatelliteInfo> m_sats
        +parseLine(QString)
        +getSatellites()
    }
    
    class AntennaTestDialog {
        -SdrStreamer* m_streamer
        -int m_state
        +startMeasurement()
        -onMeasurementTimeout()
    }
```

### Core Components & Responsibilities:

*   **`MainWindow`:** The central controller. It initializes the UI components, instantiates the worker threads, and connects Qt Signals/Slots to route data (e.g., routing `powerMeasured` from the Streamer to the UI Label). It owns the `QProcess` that manages the `gnss-sdr` lifecycle.
*   **`SdrStreamer`:** The hardware abstraction layer for streaming. It configures the LimeSDR via SoapySDR (`setSampleRate`, `setFrequency`, `setGain`), opens the Named Pipe (with `O_WRONLY | O_NONBLOCK`), and runs a tight `while(m_running)` loop pulling buffers.
*   **`NmeaParser`:** A stateless string tokenizer. It receives raw lines from the tailing operation in `MainWindow`, matches NMEA prefixes (`$GNGGA`, `$GPGSV`), splits by comma, and populates data structures representing satellites (PRN, Elevation, Azimuth, SNR).
*   **`HeadlessTuner`:** An autonomous state machine. Instead of relying on user clicks, it holds a `QList` of configurations. It starts a configuration, waits a specified duration (e.g., 40 seconds), checks the NMEA parser for a 3D fix, logs the result to a CSV, and transitions to the next configuration.
*   **`AntennaTestDialog`:** A specialized QDialog that borrows an `SdrStreamer` instance to stream data to `/dev/null` temporarily. This isolates the SDR to measure thermal noise averages in two states (plugged vs unplugged).

```

### Data Flow Sequence Diagram

This sequence diagram illustrates the complex startup sequence and data flow pipeline when a user clicks the "Start" button in the Cockpit. It highlights how IPC is established using a POSIX named pipe before the SDR begins writing.

```mermaid
sequenceDiagram
    actor User
    participant Main as "MainWindow (UI)"
    participant QTh as "SdrStreamer (QThread)"
    participant SDR as "SoapySDR (Hardware)"
    participant Pipe as "POSIX FIFO (.fifo)"
    participant GNSS as "gnss-sdr (QProcess)"

    User->>Main: Click "Start Receiver"
    Main->>Main: Generate gnss-sdr.conf
    Main->>Pipe: mkfifo(path) (Create Pipe)
    
    Main->>QTh: startStreaming(path)
    activate QTh
    QTh->>SDR: Setup & Initialize LimeSDR
    QTh->>Pipe: open(O_WRONLY, O_NONBLOCK)
    Note right of QTh: Loops until gnss-sdr attaches reader
    
    Main->>GNSS: QProcess::start("gnss-sdr")
    activate GNSS
    GNSS->>Pipe: open(O_RDONLY) (Reader attached)
    
    QTh->>Pipe: Switch to blocking mode
    
    loop Real-Time Streaming
        QTh->>SDR: readStream()
        SDR-->>QTh: CF32 IQ Samples
        QTh->>Pipe: write() samples
        Pipe-->>GNSS: Read samples
        GNSS->>GNSS: Correlate & Track (DSP)
    end
    
    GNSS-->>Main: Write PVT to .nmea file
    Main->>Main: Tail & Parse NMEA
    Main-->>User: Update Skyplot & UI Dashboard
```

### Diagnostic State Diagram (Antenna LNA Test)

The `AntennaTestDialog` employs a state machine to orchestrate the measurement of thermal noise, temporarily taking over the SDR hardware to prove the LNA is active.

```mermaid
stateDiagram-v2
    [*] --> INIT : User opens dialog
    
    INIT --> MEASURING_PLUGGED : User clicks "Start Test"
    
    state MEASURING_PLUGGED {
        direction LR
        StreamOn1 : SDR Streams to dev null
        Measure1 : Accumulate powerDb
        StreamOn1 --> Measure1 : 3 seconds
    }
    
    MEASURING_PLUGGED --> WAITING_UNPLUG : Timeout
    
    WAITING_UNPLUG --> MEASURING_UNPLUGGED : User unplugs & clicks "Ready"
    
    state MEASURING_UNPLUGGED {
        direction LR
        StreamOn2 : SDR Streams to dev null
        Measure2 : Accumulate powerDb
        StreamOn2 --> Measure2 : 3 seconds
    }
    
    MEASURING_UNPLUGGED --> FINISHED : Timeout
    
    state FINISHED {
        calc : Compare Plugged vs Unplugged
    }
    
    FINISHED --> [*] : User closes dialog
```

---

## 5. Functional Capabilities & Requirements

### 5.1 Real-Time Signal Processing
*   The system configures the SDR to sample at 2.0 MSPS (or 4.0 MSPS) centered at 1575.42 MHz (L1 band).
*   The system implements offset tuning (2 MHz) to avoid DC spike contamination in the center of the GPS band.
*   The system provides an option to enable the 3.3V Bias-Tee to power active GNSS antennas.

### 5.2 Dynamic Configuration & DSP Orchestration
*   The system dynamically generates a valid `gnss-sdr` configuration file (`gnss-sdr.conf`) matching the current hardware and GUI state (Gain, Bandwidths).
*   The system spawns `gnss-sdr` and monitors its stdout/stderr for status updates, catching pipeline errors or thread terminations.

### 5.3 Automated Headless Tuning (Grid Search)
*   A dedicated CLI mode (`--headless-tune`) bypasses the GUI to run an automated grid search.
*   The tuning engine iterates through combinations of RF Gain (45-65 dB), PLL Bandwidth (30-50 Hz), and DLL Bandwidth (1-3 Hz) to find optimal parameters for unstable clocks.
*   The tuning engine evaluates success based on the acquisition of a valid 3D NMEA fix and logs results to `tuning_results.csv`.

### 5.4 Hardware Diagnostics
*   **Antenna LNA Test:** Measures the thermal noise floor drop between a plugged and unplugged state to verify active antenna power.
*   **RF Spectrum Scanner:** Rapidly hops frequencies (e.g., 10 MHz to 3.8 GHz) to identify local out-of-band interference (like cellular or 5G bands).

---

## 6. Non-Functional Requirements

*   **Performance:** The UI thread must remain responsive at 60 FPS while the `SdrStreamer` thread processes 16 MB/s of incoming IQ data.
*   **Robustness:** The application must clean up all child processes (`gnss-sdr`) and POSIX FIFOs upon exit (SIGINT/SIGTERM) to prevent zombie processes and locked USB devices.
*   **Usability:** Complex DSP metrics (like C/N0 and lock state) must be distilled into intuitive visual representations (Polar Skyplots, color-coded status text).
