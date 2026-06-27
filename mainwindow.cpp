#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QDateTime>
#include <QFileInfo>
#include <QDir>
#include <QRegularExpression>
#include <QTextStream>
#include <QPalette>
#include <QStyle>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_sdrStreamer(new SdrStreamer(this))
    , m_gnssProcess(new QProcess(this))
    , m_currentPowerDb(-120.0)
    , m_receiverTime("0 s")
    , m_logFilePos(0)
    , m_nmeaFilePos(0)
{
    m_workspaceDir = QDir::currentPath();
    m_fifoPath = m_workspaceDir + "/limesdr_fifo.dat";
    m_configPath = m_workspaceDir + "/gnss_sdr_limesdr_temp.conf";
    m_nmeaPath = m_workspaceDir + "/gnss_output/gnss_sdr_pvt_limesdr.nmea";
    m_logPath = m_workspaceDir + "/gnss-sdr.log";

    // Setup 8 tracking channels
    for (int i = 0; i < 8; ++i) {
        m_channels[i] = ChannelState();
    }

    setupUi();
    applyTheme();

    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &MainWindow::onPeriodicUpdate);

    connect(m_sdrStreamer, &SdrStreamer::powerMeasured, this, &MainWindow::onPowerMeasured);
    connect(m_sdrStreamer, &SdrStreamer::logMessage, this, [this](const QString &msg) {
        appendLog("[SDR] " + msg, "#3b82f6");
    });
    connect(m_sdrStreamer, &SdrStreamer::finishedStreaming, this, &MainWindow::onSdrFinished);

    connect(m_gnssProcess, &QProcess::readyReadStandardOutput, this, &MainWindow::onGnssProcessReadyRead);
    connect(m_gnssProcess, &QProcess::readyReadStandardError, this, &MainWindow::onGnssProcessReadyRead);
    connect(m_gnssProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onGnssProcessFinished);

    appendLog("System ready. Connect LimeSDR, attach active antenna to RX0 (LNAH), and click Start.", "#10b981");
}

MainWindow::~MainWindow()
{
    onStopClicked();
    delete m_sdrStreamer;
}

void MainWindow::setupUi()
{
    setWindowTitle("LimeSDR GPS Cockpit Dashboard");
    resize(1024, 768);

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(12);

    // --- Tab Widget ---
    QTabWidget *tabWidget = new QTabWidget(this);
    mainLayout->addWidget(tabWidget);

    // --- DASHBOARD TAB ---
    QWidget *tabDashboard = new QWidget(this);
    tabWidget->addTab(tabDashboard, "Cockpit Dashboard");
    
    QVBoxLayout *dashLayout = new QVBoxLayout(tabDashboard);
    dashLayout->setContentsMargins(10, 10, 10, 10);
    dashLayout->setSpacing(12);

    // 1. Controls Group
    QGroupBox *ctrlGroup = new QGroupBox("Receiver Configuration && Controls", this);
    dashLayout->addWidget(ctrlGroup);
    
    QHBoxLayout *ctrlLayout = new QHBoxLayout(ctrlGroup);
    ctrlLayout->setContentsMargins(10, 10, 10, 10);
    ctrlLayout->setSpacing(15);

    m_btnStart = new QPushButton("Start GPS Receiver", this);
    m_btnStart->setFixedHeight(36);
    connect(m_btnStart, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    ctrlLayout->addWidget(m_btnStart);

    m_btnStop = new QPushButton("Stop", this);
    m_btnStop->setFixedHeight(36);
    m_btnStop->setEnabled(false);
    connect(m_btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    ctrlLayout->addWidget(m_btnStop);

    // Gain control
    ctrlLayout->addWidget(new QLabel("Gain:", this));
    m_sliderGain = new QSlider(Qt::Horizontal, this);
    m_sliderGain->setRange(0, 73); // LimeSDR USB RX LNA/PGA/TIA combination up to 73 dB
    m_sliderGain->setValue(60);
    m_sliderGain->setFixedWidth(150);
    connect(m_sliderGain, &QSlider::valueChanged, this, &MainWindow::onGainSliderChanged);
    ctrlLayout->addWidget(m_sliderGain);

    m_lblGainVal = new QLabel("60 dB", this);
    ctrlLayout->addWidget(m_lblGainVal);

    // Sample rate control
    ctrlLayout->addWidget(new QLabel("Sample Rate:", this));
    m_comboSampleRate = new QComboBox(this);
    m_comboSampleRate->addItems({"2.0 MSPS (GPS L1 Optimized)", "4.0 MSPS"});
    m_comboSampleRate->setCurrentIndex(0);
    ctrlLayout->addWidget(m_comboSampleRate);

    // Bias Tee control
    m_chkBiasTee = new QCheckBox("Enable Active Antenna Bias-Tee (3.3V)", this);
    m_chkBiasTee->setChecked(false); // default off to prevent damage to passive setups
    ctrlLayout->addWidget(m_chkBiasTee);

    // Adaptive Gain control
    m_chkAdaptiveGain = new QCheckBox("Adaptive Gain Tuning", this);
    m_chkAdaptiveGain->setChecked(true); // default ON
    m_chkAdaptiveGain->setToolTip("Automatically adjust RF gain to maintain optimal ADC power level (-20 to -15 dBFS)");
    ctrlLayout->addWidget(m_chkAdaptiveGain);

    ctrlLayout->addStretch();

    // 2. Status Ribbon
    QWidget *statusRibbon = new QWidget(this);
    statusRibbon->setObjectName("statusRibbon");
    dashLayout->addWidget(statusRibbon);
    
    QHBoxLayout *ribbonLayout = new QHBoxLayout(statusRibbon);
    ribbonLayout->setContentsMargins(12, 8, 12, 8);
    
    m_lblStatus = new QLabel("RECEIVER STATE: STOPPED", this);
    m_lblStatus->setObjectName("lblStatus");
    ribbonLayout->addWidget(m_lblStatus);
    
    ribbonLayout->addStretch();
    
    m_lblPower = new QLabel("RF POWER: N/A", this);
    m_lblPower->setObjectName("lblPower");
    ribbonLayout->addWidget(m_lblPower);
    
    ribbonLayout->addStretch();
    
    m_lblTime = new QLabel("RCV TIME: 0 s", this);
    m_lblTime->setObjectName("lblTime");
    ribbonLayout->addWidget(m_lblTime);

    // 3. Middle Section: PVT Info (Left) && C/N0 Plot (Right)
    QHBoxLayout *midLayout = new QHBoxLayout();
    dashLayout->addLayout(midLayout);

    // PVT Group Box
    QGroupBox *pvtGroup = new QGroupBox("GPS Position Velocity Time (PVT)", this);
    pvtGroup->setFixedWidth(400);
    midLayout->addWidget(pvtGroup);
    
    QFormLayout *pvtLayout = new QFormLayout(pvtGroup);
    pvtLayout->setSpacing(8);
    pvtLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_lblLatitude = new QLabel("---", this);
    m_lblLatitude->setObjectName("coordVal");
    pvtLayout->addRow("Latitude:", m_lblLatitude);

    m_lblLongitude = new QLabel("---", this);
    m_lblLongitude->setObjectName("coordVal");
    pvtLayout->addRow("Longitude:", m_lblLongitude);

    m_lblAltitude = new QLabel("---", this);
    m_lblAltitude->setObjectName("coordVal");
    pvtLayout->addRow("Altitude:", m_lblAltitude);

    m_lblSpeed = new QLabel("---", this);
    m_lblSpeed->setObjectName("statusVal");
    pvtLayout->addRow("Speed Over Ground:", m_lblSpeed);

    m_lblFixQuality = new QLabel("Searching...", this);
    m_lblFixQuality->setObjectName("statusVal");
    pvtLayout->addRow("Fix Quality:", m_lblFixQuality);

    m_lblSatsInFix = new QLabel("0", this);
    m_lblSatsInFix->setObjectName("statusVal");
    pvtLayout->addRow("Sats in Fix:", m_lblSatsInFix);

    m_lblHdop = new QLabel("---", this);
    m_lblHdop->setObjectName("statusVal");
    pvtLayout->addRow("HDOP:", m_lblHdop);

    m_lblUtcTime = new QLabel("---", this);
    m_lblUtcTime->setObjectName("timeVal");
    pvtLayout->addRow("UTC Time:", m_lblUtcTime);

    m_lblLocalTime = new QLabel("---", this);
    m_lblLocalTime->setObjectName("timeVal");
    pvtLayout->addRow("Local Time:", m_lblLocalTime);

    // Satellite C/N0 (SNR) Visualizer Group Box
    QGroupBox *satsGroup = new QGroupBox("Satellite Signals (C/N0 dB-Hz)", this);
    midLayout->addWidget(satsGroup);
    
    QVBoxLayout *satsLayout = new QVBoxLayout(satsGroup);
    m_satsContainer = new QWidget(this);
    satsLayout->addWidget(m_satsContainer);
    
    QHBoxLayout *satsBarLayout = new QHBoxLayout(m_satsContainer);
    satsBarLayout->setContentsMargins(10, 10, 10, 10);
    satsBarLayout->setSpacing(8);
    // Initial placeholder label
    QLabel *satsPlaceholder = new QLabel("Waiting for satellite acquisition...", this);
    satsPlaceholder->setAlignment(Qt::AlignCenter);
    satsBarLayout->addWidget(satsPlaceholder);

    // 4. Bottom Section: Channel Status (8 Channels Grid)
    QGroupBox *chanGroup = new QGroupBox("Channel Tracking Status", this);
    dashLayout->addWidget(chanGroup);
    
    QVBoxLayout *chanLayout = new QVBoxLayout(chanGroup);
    m_channelsContainer = new QWidget(this);
    chanLayout->addWidget(m_channelsContainer);
    
    QGridLayout *gridChan = new QGridLayout(m_channelsContainer);
    gridChan->setContentsMargins(5, 5, 5, 5);
    gridChan->setSpacing(10);

    // Initialize 8 channel cards layout
    updateChannelsDisplay();

    // --- SYSTEM CONSOLE TAB ---
    QWidget *tabConsole = new QWidget(this);
    tabWidget->addTab(tabConsole, "System Console");
    
    QVBoxLayout *consoleLayout = new QVBoxLayout(tabConsole);
    m_txtConsole = new QTextEdit(this);
    m_txtConsole->setReadOnly(true);
    m_txtConsole->setObjectName("txtConsole");
    consoleLayout->addWidget(m_txtConsole);
}

void MainWindow::applyTheme()
{
    // Apply styling for a sleek dark theme
    QString style = R"(
        QMainWindow {
            background-color: #1a1e24;
            color: #e2e8f0;
            font-family: 'Inter', 'SF Pro Text', -apple-system, sans-serif;
        }
        QGroupBox {
            border: 1px solid #2d3748;
            border-radius: 8px;
            margin-top: 12px;
            font-weight: bold;
            color: #3b82f6;
            background-color: #212630;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 10px;
            padding: 0 5px;
        }
        QLabel {
            color: #94a3b8;
            font-size: 13px;
        }
        #coordVal {
            font-size: 20px;
            font-family: 'Fira Code', 'Courier New', monospace;
            font-weight: bold;
            color: #10b981;
        }
        #statusVal {
            font-size: 14px;
            font-weight: bold;
            color: #e2e8f0;
        }
        #timeVal {
            font-size: 13px;
            font-family: 'Fira Code', 'Courier New', monospace;
            color: #f59e0b;
        }
        QPushButton {
            background-color: #3b82f6;
            color: white;
            border: none;
            border-radius: 6px;
            padding: 6px 16px;
            font-weight: bold;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1d4ed8;
        }
        QPushButton:disabled {
            background-color: #4b5563;
            color: #9ca3af;
        }
        QComboBox {
            background-color: #2d3748;
            color: #e2e8f0;
            border: 1px solid #4a5568;
            border-radius: 6px;
            padding: 4px;
        }
        QCheckBox {
            color: #cbd5e1;
        }
        #statusRibbon {
            background-color: #1e2533;
            border: 1px solid #2d3748;
            border-radius: 8px;
        }
        #lblStatus {
            font-size: 14px;
            font-weight: bold;
            color: #ef4444; /* Initial Stopped */
        }
        #lblPower {
            font-size: 14px;
            font-weight: bold;
            color: #3b82f6;
        }
        #lblTime {
            font-size: 14px;
            font-weight: bold;
            color: #f59e0b;
        }
        #txtConsole {
            background-color: #0f172a;
            color: #cbd5e1;
            font-family: 'Fira Code', 'Courier New', monospace;
            font-size: 12px;
            border: 1px solid #334155;
            border-radius: 8px;
        }
        QProgressBar {
            background-color: #1e293b;
            border: 1px solid #334155;
            border-radius: 4px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: #10b981;
        }
        /* Channel Card styling */
        .ChannelCard {
            background-color: #1e293b;
            border: 1px solid #334155;
            border-radius: 6px;
            padding: 8px;
        }
        .ChannelHeader {
            font-size: 11px;
            font-weight: bold;
            color: #64748b;
        }
        .ChannelSat {
            font-size: 14px;
            font-weight: bold;
            color: #e2e8f0;
        }
        .ChannelStatus {
            font-size: 11px;
            font-weight: bold;
        }
    )";
    setStyleSheet(style);
}

void MainWindow::setupFifo()
{
    QFile::remove(m_fifoPath);
    if (::mkfifo(m_fifoPath.toUtf8().constData(), 0666) < 0) {
        appendLog("ERROR: mkfifo failed: " + QString(strerror(errno)), "#ef4444");
    } else {
        appendLog("FIFO Pipe created successfully at " + m_fifoPath, "#10b981");
    }
}

void MainWindow::writeGnssSdrConfig()
{
    // Write out the configuration file for gnss-sdr pointing to FIFO
    QFile file(m_configPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        appendLog("ERROR: Failed to write config file!", "#ef4444");
        return;
    }

    QTextStream out(&file);
    out << "; Temporary configuration generated by LimeSDR GPS Cockpit\n";
    out << "[GNSS-SDR]\n";
    out << "GNSS-SDR.internal_fs_sps=2000000\n";
    out << "GNSS-SDR.telecommand_enabled=false\n";
    out << "GNSS-SDR.init_latitude_deg=13.7563\n";
    out << "GNSS-SDR.init_longitude_deg=100.5018\n";
    out << "GNSS-SDR.init_altitude_m=10\n";
    out << "GNSS-SDR.SUPL_gps_enabled=true\n";
    out << "GNSS-SDR.SUPL_read_gps_assistance_xml=true\n\n";

    out << "SignalSource.implementation=Fifo_Signal_Source\n";
    out << "SignalSource.item_type=gr_complex\n";
    out << "SignalSource.filename=" << m_fifoPath << "\n";
    out << "SignalSource.sampling_frequency=2000000\n";
    out << "SignalSource.antenna=LNAH\n";
    out << "SignalSource.gain=60\n";
    out << "SignalSource.dump=false\n\n";

    out << "SignalConditioner.implementation=Pass_Through\n\n";

    out << "Channels_1C.count=8\n";
    out << "Channels.in_acquisition=4\n\n"; // 4 parallel acquisition units to find satellites twice as fast

    out << "Channel0.signal=1C\n";
    out << "Channel1.signal=1C\n";
    out << "Channel2.signal=1C\n";
    out << "Channel3.signal=1C\n";
    out << "Channel4.signal=1C\n";
    out << "Channel5.signal=1C\n";
    out << "Channel6.signal=1C\n";
    out << "Channel7.signal=1C\n\n";

    out << "Acquisition_1C.implementation=GPS_L1_CA_PCPS_Acquisition\n";
    out << "Acquisition_1C.item_type=gr_complex\n";
    out << "Acquisition_1C.coherent_integration_time_ms=1\n";
    out << "Acquisition_1C.threshold=2.0\n"; // 2.0 threshold for high sensitivity on weak signals
    out << "Acquisition_1C.doppler_max=5000\n";
    out << "Acquisition_1C.doppler_step=250\n";
    out << "Acquisition_1C.max_dwells=10\n"; // 10 dwells to average noise and avoid false drops
    out << "Acquisition_1C.blocking=true\n";
    out << "Acquisition_1C.repeat_satellite=true\n";
    out << "Acquisition_1C.dump=false\n\n";

    out << "Tracking_1C.implementation=GPS_L1_CA_DLL_PLL_Tracking\n";
    out << "Tracking_1C.item_type=gr_complex\n";
    out << "Tracking_1C.pll_bw_hz=40.0\n"; // 40 Hz pull-in bandwidth to catch carrier offset
    out << "Tracking_1C.dll_bw_hz=2.0\n";
    out << "Tracking_1C.pull_in_time_ms=30000\n"; // 30s pull-in duration to allow frequency loops to settle
    out << "Tracking_1C.pll_bw_narrow_hz=30.0\n"; // Wider steady-state PLL bandwidth (30 Hz) to tolerate LimeSDR phase noise
    out << "Tracking_1C.dll_bw_narrow_hz=2.0\n";
    out << "Tracking_1C.early_late_space_chips=0.5\n";
    out << "Tracking_1C.early_late_space_narrow_chips=0.25\n";
    out << "Tracking_1C.enable_fll_pull_in=true\n";
    out << "Tracking_1C.fll_bw_hz=10.0\n";
    out << "Tracking_1C.enable_fll_steady_state=true\n"; // FLL steady state enabled to prevent phase tracking slips
    out << "Tracking_1C.order=3\n"; // 3rd order loop dynamically tracks Doppler rate
    out << "Tracking_1C.cn0_min=0\n";
    out << "Tracking_1C.max_lock_fail=100000\n"; // Long lock failure threshold (100k samples = 100 seconds)
    out << "Tracking_1C.carrier_lock_th=0.01\n"; // Relaxed phase coherence threshold (0.01) to stay locked on weak signals
    out << "Tracking_1C.dump=false\n\n";

    out << "TelemetryDecoder_1C.implementation=GPS_L1_CA_Telemetry_Decoder\n";
    out << "TelemetryDecoder_1C.dump=false\n\n";

    out << "Observables.implementation=Hybrid_Observables\n";
    out << "Observables.dump=false\n\n";

    out << "PVT.implementation=RTKLIB_PVT\n";
    out << "PVT.positioning_mode=Single\n";
    out << "PVT.output_rate_ms=1000\n";
    out << "PVT.display_rate_ms=500\n";
    out << "PVT.nmea_dump_filename=gnss_sdr_pvt_limesdr.nmea\n";
    out << "PVT.flag_nmea_tty_port=false\n";
    out << "PVT.flag_rtcm_server=false\n";
    out << "PVT.flag_rtcm_tty_port=false\n";
    out << "PVT.dump=false\n";
    out << "PVT.kml_output_enabled=true\n";
    out << "PVT.gpx_output_enabled=true\n";
    out << "PVT.nmea_output_file_enabled=true\n";
    out << "PVT.rinex_output_enabled=false\n";
    out << "PVT.output_path=" << m_workspaceDir + "/gnss_output/" << "\n";

    file.close();
    appendLog("Config file written dynamically: " + m_configPath);
}

void MainWindow::onStartClicked()
{
    appendLog("Starting GPS Receiver flow...", "#3b82f6");

    m_btnStart->setEnabled(false);
    m_comboSampleRate->setEnabled(false);
    m_chkBiasTee->setEnabled(false);
    
    // Reset indicators
    m_lblStatus->setText("RECEIVER STATE: CONNECTING");
    m_lblStatus->setStyleSheet("color: #f59e0b;"); // yellow

    // Clean previous output files and paths
    QDir().mkpath(m_workspaceDir + "/gnss_output");
    QFile::remove(m_nmeaPath);
    QFile::remove(m_logPath);

    setupFifo();
    writeGnssSdrConfig();

    // Reset parameters
    m_currentPowerDb = -120.0;
    m_receiverTime = "0 s";
    m_nmeaParser.clear();
    m_logFilePos = 0;
    m_nmeaFilePos = 0;
    m_startTime = QDateTime::currentMSecsSinceEpoch();

    for (int i = 0; i < 8; ++i) {
        m_channels[i] = ChannelState();
    }
    updateChannelsDisplay();
    updateSatellitesDisplay();

    // 1. Start SDR Streaming Thread
    double rate = 2.0e6;
    if (m_comboSampleRate->currentIndex() == 1) {
        rate = 4.0e6;
    }
    double gain = m_sliderGain->value();
    bool enableBiasTee = m_chkBiasTee->isChecked();

    m_sdrStreamer->startStreaming(m_fifoPath, rate, gain, enableBiasTee);

    // 2. Start GNSS-SDR Process
    appendLog("Launching GNSS-SDR process...", "#3b82f6");
    QStringList args;
    args << "--config_file=" + m_configPath;
    args << "--log_dir=" + m_workspaceDir;

    // Use environment variables for immediate logging flush
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("GLOG_logbuflevel", "-1");
    m_gnssProcess->setProcessEnvironment(env);

    m_gnssProcess->setWorkingDirectory(m_workspaceDir);

    QString gnssSdrPath = "gnss-sdr";
    if (QFile::exists("/opt/local/bin/gnss-sdr")) {
        gnssSdrPath = "/opt/local/bin/gnss-sdr";
    } else if (QFile::exists("/usr/local/bin/gnss-sdr")) {
        gnssSdrPath = "/usr/local/bin/gnss-sdr";
    } else if (QFile::exists("/usr/bin/gnss-sdr")) {
        gnssSdrPath = "/usr/bin/gnss-sdr";
    }
    
    appendLog("Launching GNSS-SDR process: " + gnssSdrPath, "#3b82f6");
    m_gnssProcess->start(gnssSdrPath, args);

    if (!m_gnssProcess->waitForStarted(2000)) {
        appendLog("ERROR: Failed to launch gnss-sdr at " + gnssSdrPath + ". Is it installed?", "#ef4444");
        onStopClicked();
        return;
    }

    m_lblStatus->setText("RECEIVER STATE: RUNNING");
    m_lblStatus->setStyleSheet("color: #10b981;"); // green
    m_btnStop->setEnabled(true);

    // Start UI update timer
    m_updateTimer->start(200);
}

void MainWindow::onStopClicked()
{
    appendLog("Stopping GPS Receiver...", "#ef4444");

    m_updateTimer->stop();

    // Stop GNSS-SDR
    if (m_gnssProcess->state() != QProcess::NotRunning) {
        m_gnssProcess->terminate();
        if (!m_gnssProcess->waitForFinished(3000)) {
            m_gnssProcess->kill();
        }
    }

    // Stop SDR streamer thread
    if (m_sdrStreamer->isRunningStream()) {
        m_sdrStreamer->stopStreaming();
        m_sdrStreamer->wait();
    }

    // Clean FIFO and config files
    QFile::remove(m_fifoPath);
    QFile::remove(m_configPath);

    m_lblStatus->setText("RECEIVER STATE: STOPPED");
    m_lblStatus->setStyleSheet("color: #ef4444;"); // red
    m_lblPower->setText("RF POWER: N/A");

    m_btnStart->setEnabled(true);
    m_btnStop->setEnabled(false);
    m_comboSampleRate->setEnabled(true);
    m_chkBiasTee->setEnabled(true);

    appendLog("GPS Receiver stopped.", "#ef4444");
}

void MainWindow::onGainSliderChanged(int value)
{
    m_lblGainVal->setText(QString("%1 dB").arg(value));
    if (m_sdrStreamer->isRunningStream()) {
        m_sdrStreamer->setDynamicGain(value);
    }
}

void MainWindow::onPowerMeasured(double powerDb)
{
    m_currentPowerDb = powerDb;
    m_lblPower->setText(QString("RF POWER: %1 dBFS").arg(powerDb, 0, 'f', 1));
    // Color alert for saturation
    if (powerDb > -5.0) {
        m_lblPower->setStyleSheet("color: #ef4444; font-weight: bold;"); // Red (saturation)
    } else {
        m_lblPower->setStyleSheet("color: #3b82f6; font-weight: bold;"); // Blue
    }

    // Adaptive Gain Tuning
    if (m_chkAdaptiveGain->isChecked() && m_sdrStreamer->isRunningStream()) {
        int currentGain = m_sliderGain->value();
        int targetGain = currentGain;

        if (powerDb > -15.0) {
            // Signal is too strong (danger of clipping/saturation)
            if (powerDb > -5.0) {
                targetGain -= 5; // Fast back-off
            } else {
                targetGain -= 1; // Gentle back-off
            }
        } else if (powerDb < -22.0) {
            // Signal is too weak (quantization noise dominating)
            targetGain += 1; // Gentle increase
        }

        // Clamp to LimeSDR RX gain limits
        targetGain = qBound(0, targetGain, 73);

        if (targetGain != currentGain) {
            m_sliderGain->blockSignals(true);
            m_sliderGain->setValue(targetGain);
            m_lblGainVal->setText(QString("%1 dB").arg(targetGain));
            m_sliderGain->blockSignals(false);
            
            // Apply dynamically to running thread
            m_sdrStreamer->setDynamicGain(targetGain);
            appendLog(QString("[AGC] Adaptive Gain adjusted to %1 dB (Power: %2 dBFS)").arg(targetGain).arg(powerDb, 0, 'f', 1), "#a78bfa");
        }
    }
}

void MainWindow::onSdrLogMessage(const QString &message)
{
    appendLog(message);
}

void MainWindow::onSdrFinished()
{
    appendLog("SDR Streamer Thread finished.");
}

void MainWindow::onGnssProcessReadyRead()
{
    QByteArray stdOut = m_gnssProcess->readAllStandardOutput();
    QByteArray stdErr = m_gnssProcess->readAllStandardError();
    QByteArray data = stdOut + stdErr;
    
    QString text = QString::fromUtf8(data);
    QStringList lines = text.split('\n');
    for (const QString &line : lines) {
        if (!line.trimmed().isEmpty()) {
            // Check for receiver time logs
            if (line.contains("Current receiver time:")) {
                QRegularExpression re("Current receiver time: (.+)");
                QRegularExpressionMatch match = re.match(line);
                if (match.hasMatch()) {
                    m_receiverTime = match.captured(1);
                    m_lblTime->setText("RCV TIME: " + m_receiverTime);
                }
            }
            appendLog("[GNSS-SDR] " + line.trimmed());
        }
    }
}

void MainWindow::onGnssProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    appendLog(QString("GNSS-SDR process exited with code %1 (status: %2)")
              .arg(exitCode).arg(exitStatus == QProcess::NormalExit ? "Normal" : "Crash"), "#ef4444");
    onStopClicked();
}

void MainWindow::onPeriodicUpdate()
{
    readGnssLog();
    readNmeaFile();
    updateChannelsDisplay();
    updateSatellitesDisplay();
}

void MainWindow::readGnssLog()
{
    // Read the gnss-sdr.log to extract channel status in real-time
    QFile file(m_logPath);
    if (!file.exists()) return;

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    if (m_logFilePos > file.size()) {
        m_logFilePos = 0; // File was truncated
    }

    file.seek(m_logFilePos);
    QTextStream in(&file);
    QString line;
    
    QRegularExpression reAcq("Successful acquisition in channel (\\d+) for satellite [A-Z]\\s*(\\d+)");
    QRegularExpression rePull("Pull-in:.*PRN (\\d+).*in channel (\\d+)");
    QRegularExpression reLoss("Loss of lock in channel (\\d+)");
    QRegularExpression reIdle("Channel (\\d+) Idle state");

    qint64 now = QDateTime::currentMSecsSinceEpoch();

    while (in.readLineInto(&line)) {
        QRegularExpressionMatch matchAcq = reAcq.match(line);
        if (matchAcq.hasMatch()) {
            int ch = matchAcq.captured(1).toInt();
            int prn = matchAcq.captured(2).toInt();
            if (m_channels.contains(ch)) {
                m_channels[ch].prn = prn;
                m_channels[ch].status = "Acquired";
                m_channels[ch].lastUpdate = now;
                m_channels[ch].lockDuration = 0;
            }
            continue;
        }

        QRegularExpressionMatch matchPull = rePull.match(line);
        if (matchPull.hasMatch()) {
            int prn = matchPull.captured(1).toInt();
            int ch = matchPull.captured(2).toInt();
            if (m_channels.contains(ch)) {
                m_channels[ch].prn = prn;
                m_channels[ch].status = "Tracking";
                m_channels[ch].lastUpdate = now;
            }
            continue;
        }

        QRegularExpressionMatch matchLoss = reLoss.match(line);
        if (matchLoss.hasMatch()) {
            int ch = matchLoss.captured(1).toInt();
            if (m_channels.contains(ch)) {
                m_channels[ch].status = "Loss of Lock";
            }
            continue;
        }

        QRegularExpressionMatch matchIdle = reIdle.match(line);
        if (matchIdle.hasMatch()) {
            int ch = matchIdle.captured(1).toInt();
            if (m_channels.contains(ch)) {
                m_channels[ch].prn = 0;
                m_channels[ch].status = "Searching";
                m_channels[ch].lockDuration = 0;
            }
            continue;
        }
    }

    m_logFilePos = file.pos();
    file.close();

    // Update lock durations
    for (auto it = m_channels.begin(); it != m_channels.end(); ++it) {
        if (it.value().status == "Tracking") {
            it.value().lockDuration = (now - it.value().lastUpdate) / 1000;
        }
    }
}

void MainWindow::readNmeaFile()
{
    // Read and parse coordinates from NMEA output
    QFile file(m_nmeaPath);
    if (!file.exists()) return;

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    if (m_nmeaFilePos > file.size()) {
        m_nmeaFilePos = 0; // File was truncated
    }

    file.seek(m_nmeaFilePos);
    QTextStream in(&file);
    QString line;
    bool parsedAny = false;

    while (in.readLineInto(&line)) {
        if (m_nmeaParser.parseLine(line)) {
            parsedAny = true;
            qDebug() << "[NMEA]" << line.trimmed();
            appendLog("[NMEA] " + line.trimmed(), "#10b981");
        }
    }

    m_nmeaFilePos = file.pos();
    file.close();

    if (parsedAny) {
        GpsPosition pos = m_nmeaParser.getPosition();
        
        if (pos.hasFix) {
            m_lblLatitude->setText(QString("%1°").arg(pos.latitude, 0, 'f', 8));
            m_lblLongitude->setText(QString("%1°").arg(pos.longitude, 0, 'f', 8));
            m_lblAltitude->setText(QString("%1 m").arg(pos.altitude, 0, 'f', 2));
            m_lblSpeed->setText(QString("%1 knots (%2 km/h)")
                                .arg(pos.speedKnots, 0, 'f', 1)
                                .arg(pos.speedKnots * 1.852, 0, 'f', 1));
            
            QString qualityText = "GPS Fix";
            QString color = "#10b981"; // green
            if (pos.fixQuality == 2) { qualityText = "DGPS Fix"; color = "#3b82f6"; }
            else if (pos.fixQuality == 3) { qualityText = "PPS Fix"; color = "#8b5cf6"; }
            else if (pos.fixQuality == 4) { qualityText = "RTK Int (Premium)"; color = "#10b981"; }
            else if (pos.fixQuality == 5) { qualityText = "RTK Float"; color = "#f59e0b"; }
            else if (pos.fixQuality == 6) { qualityText = "Estimated"; color = "#6b7280"; }

            m_lblFixQuality->setText(qualityText);
            m_lblFixQuality->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color));
        } else {
            m_lblLatitude->setText("---");
            m_lblLongitude->setText("---");
            m_lblAltitude->setText("---");
            m_lblSpeed->setText("---");
            m_lblFixQuality->setText("Searching for lock...");
            m_lblFixQuality->setStyleSheet("color: #ef4444; font-weight: bold;");
        }

        m_lblSatsInFix->setText(QString::number(pos.numSatsInFix));
        m_lblHdop->setText(pos.hdop > 99.0 ? "---" : QString::number(pos.hdop, 'f', 2));

        if (pos.utcDateTime.isValid()) {
            m_lblUtcTime->setText(pos.utcDateTime.toString("yyyy-MM-dd hh:mm:ss t"));
            m_lblLocalTime->setText(pos.utcDateTime.toLocalTime().toString("yyyy-MM-dd hh:mm:ss t"));
        }
    }
}

void MainWindow::updateChannelsDisplay()
{
    // Clean layout
    QLayoutItem *child;
    while ((child = m_channelsContainer->layout()->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }

    QGridLayout *grid = qobject_cast<QGridLayout*>(m_channelsContainer->layout());
    if (!grid) return;

    for (int i = 0; i < 8; ++i) {
        QWidget *card = new QWidget(m_channelsContainer);
        card->setObjectName(QString("card%1").arg(i));
        card->setProperty("class", "ChannelCard");
        
        QVBoxLayout *cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(6, 6, 6, 6);
        cardLayout->setSpacing(2);

        QLabel *lblChan = new QLabel(QString("CHANNEL %1").arg(i), card);
        lblChan->setProperty("class", "ChannelHeader");
        cardLayout->addWidget(lblChan);

        ChannelState state = m_channels[i];
        QString satText = "Searching...";
        QString statusText = "SEARCHING";
        QString statusColor = "#64748b"; // gray
        QString lockTimeStr = "";

        if (state.prn > 0) {
            satText = QString("GPS PRN %1").arg(state.prn, 2, 10, QChar('0'));
            
            if (state.status == "Tracking") {
                statusText = "LOCKED";
                statusColor = "#10b981"; // green
                lockTimeStr = QString("%1s lock").arg(state.lockDuration);
            } else if (state.status == "Acquired") {
                statusText = "ACQUIRED";
                statusColor = "#f59e0b"; // yellow
            } else if (state.status == "Loss of Lock") {
                statusText = "LOCK LOSS";
                statusColor = "#ef4444"; // red
            }
        }

        QLabel *lblSat = new QLabel(satText, card);
        lblSat->setProperty("class", "ChannelSat");
        cardLayout->addWidget(lblSat);

        QLabel *lblStatus = new QLabel(statusText, card);
        lblStatus->setProperty("class", "ChannelStatus");
        lblStatus->setStyleSheet(QString("color: %1;").arg(statusColor));
        cardLayout->addWidget(lblStatus);

        QLabel *lblLockTime = new QLabel(lockTimeStr, card);
        lblLockTime->setStyleSheet("font-size: 10px; color: #94a3b8;");
        cardLayout->addWidget(lblLockTime);

        // Styling the card container based on status
        QString borderStyle = "border: 1px solid #334155;";
        if (statusText == "LOCKED") borderStyle = "border: 1px solid #047857; background-color: #064e3b;";
        else if (statusText == "ACQUIRED") borderStyle = "border: 1px solid #b45309; background-color: #78350f;";
        else if (statusText == "LOCK LOSS") borderStyle = "border: 1px solid #b91c1c; background-color: #7f1d1d;";
        card->setStyleSheet(QString(".ChannelCard { %1 border-radius: 6px; }").arg(borderStyle));

        int row = i / 4;
        int col = i % 4;
        grid->addWidget(card, row, col);
    }
}

void MainWindow::updateSatellitesDisplay()
{
    // Clear layout
    QLayoutItem *child;
    while ((child = m_satsContainer->layout()->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }

    QHBoxLayout *barLayout = qobject_cast<QHBoxLayout*>(m_satsContainer->layout());
    if (!barLayout) return;

    QMap<int, int> satSnr = m_nmeaParser.getSatelliteSnr();

    // Merge active tracking channel PRNs into satSnr if not already present from NMEA
    for (int i = 0; i < 8; ++i) {
        if (m_channels[i].prn > 0) {
            int prn = m_channels[i].prn;
            if (!satSnr.contains(prn)) {
                if (m_channels[i].status == "Tracking") {
                    satSnr[prn] = 40; // Est SNR for tracking
                } else if (m_channels[i].status == "Acquired") {
                    satSnr[prn] = 35; // Est SNR for acquired
                }
            }
        }
    }

    // Filter out PRNs that are currently tracked on channels to color code them green
    QSet<int> trackedPrns;
    for (int i = 0; i < 8; ++i) {
        if (m_channels[i].status == "Tracking" && m_channels[i].prn > 0) {
            trackedPrns.insert(m_channels[i].prn);
        }
    }

    if (satSnr.isEmpty()) {
        QLabel *lblEmpty = new QLabel("Waiting for satellite signals...", m_satsContainer);
        lblEmpty->setAlignment(Qt::AlignCenter);
        barLayout->addWidget(lblEmpty);
        return;
    }

    // Sort by PRN number
    QList<int> prns = satSnr.keys();
    std::sort(prns.begin(), prns.end());

    // Show up to 10 satellites horizontally
    int count = 0;
    for (int prn : prns) {
        if (count >= 12) break;
        
        int snr = satSnr[prn];

        QWidget *barWidget = new QWidget(m_satsContainer);
        QVBoxLayout *barVLayout = new QVBoxLayout(barWidget);
        barVLayout->setContentsMargins(2, 2, 2, 2);
        barVLayout->setSpacing(4);

        QLabel *lblPrn = new QLabel(QString("P%1").arg(prn, 2, 10, QChar('0')), barWidget);
        lblPrn->setAlignment(Qt::AlignCenter);
        lblPrn->setStyleSheet("font-weight: bold; font-family: monospace; font-size: 11px; color: #e2e8f0;");
        barVLayout->addWidget(lblPrn);

        QProgressBar *bar = new QProgressBar(barWidget);
        bar->setOrientation(Qt::Vertical);
        bar->setRange(0, 50); // Typical C/N0 is 30 to 50 dB-Hz for lock
        bar->setValue(snr);
        bar->setTextVisible(false); // Hide the default Qt percentage text (e.g. "80%")
        bar->setFixedHeight(120);
        bar->setFixedWidth(20);
        
        // Color custom progress bars
        bool isTracked = trackedPrns.contains(prn);
        QString chunkColor = isTracked ? "#10b981" : "#f59e0b"; // Green if tracked, orange if view-only
        bar->setStyleSheet(QString(
            "QProgressBar {"
            "  border: 1px solid #334155;"
            "  background-color: #0f172a;"
            "  border-radius: 2px;"
            "}"
            "QProgressBar::chunk {"
            "  background-color: %1;"
            "}"
        ).arg(chunkColor));

        barVLayout->addWidget(bar);

        QLabel *lblSnr = new QLabel(QString("%1").arg(snr), barWidget);
        lblSnr->setAlignment(Qt::AlignCenter);
        lblSnr->setStyleSheet("font-size: 10px; font-family: monospace; color: #94a3b8;");
        barVLayout->addWidget(lblSnr);

        barLayout->addWidget(barWidget);
        count++;
    }
    barLayout->addStretch();
}

void MainWindow::appendLog(const QString &msg, const QString &color)
{
    // Print to terminal stdout
    qDebug() << QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"), msg);

    // Write log with a timestamp
    QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString formattedMsg = QString("<span style=\"color: #64748b;\">[%1]</span> <span style=\"color: %2;\">%3</span>")
                           .arg(timeStr, color, msg.toHtmlEscaped());
    
    // Convert newlines in raw logs nicely
    formattedMsg.replace("\n", "<br>");
    
    m_txtConsole->append(formattedMsg);
}
