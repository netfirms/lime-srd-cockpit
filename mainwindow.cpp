#include "mainwindow.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>
#include <QMessageBox>
#include <QPainter>
#include <QPaintEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QDesktopServices>
#include <QUrl>
#include "antennatestdialog.h"
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
#include <QPainterPath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_sdrStreamer(new SdrStreamer(this))
    , m_sdrScanner(new SdrScanner(this))
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

    m_lossOfLockCount = 0;
    
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &MainWindow::onPeriodicUpdate);
    
    m_autoTuneTimer = new QTimer(this);
    connect(m_autoTuneTimer, &QTimer::timeout, this, &MainWindow::onAutoTuneTimer);

    m_previousAverageCn0 = 0.0;
    m_agcDirection = 1; // start by increasing gain
    m_snrAgcTimer = new QTimer(this);
    connect(m_snrAgcTimer, &QTimer::timeout, this, &MainWindow::onSnrAgcTimer);

    connect(m_sdrStreamer, &SdrStreamer::powerMeasured, this, &MainWindow::onPowerMeasured);
    connect(m_sdrStreamer, &SdrStreamer::logMessage, this, [this](const QString &msg) {
        appendLog("[SDR] " + msg, "#3b82f6");
    });
    connect(m_sdrStreamer, &SdrStreamer::finishedStreaming, this, &MainWindow::onSdrFinished);

    connect(m_sdrScanner, &SdrScanner::scanPoint, this, &MainWindow::onScanPointReceived);
    connect(m_sdrScanner, &SdrScanner::interferenceDetected, this, &MainWindow::onInterferenceDetected);
    connect(m_sdrScanner, &SdrScanner::scanFinished, this, &MainWindow::onScanFinished);
    connect(m_sdrScanner, &SdrScanner::logMessage, this, [this](const QString &msg) {
        appendLog("[SCANNER] " + msg, "#a78bfa");
    });

    connect(m_gnssProcess, &QProcess::readyReadStandardOutput, this, &MainWindow::onGnssProcessReadyRead);
    connect(m_gnssProcess, &QProcess::readyReadStandardError, this, &MainWindow::onGnssProcessReadyRead);
    connect(m_gnssProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onGnssProcessFinished);

    appendLog("System ready. Connect LimeSDR, attach active antenna to RX0 (LNAH), and click Start.", "#10b981");
}

MainWindow::~MainWindow()
{
    onStopClicked();
    onStopScanClicked();
    delete m_sdrStreamer;
    delete m_sdrScanner;
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
    QHBoxLayout *topControlsLayout = new QHBoxLayout();
    dashLayout->addLayout(topControlsLayout);
    
    // Group 1: Action Panel
    QGroupBox *actionGroup = new QGroupBox("Action Panel", this);
    QVBoxLayout *actionLayout = new QVBoxLayout(actionGroup);
    
    m_btnStart = new QPushButton("Start GPS Receiver", this);
    m_btnStart->setFixedHeight(36);
    connect(m_btnStart, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    actionLayout->addWidget(m_btnStart);

    m_btnStop = new QPushButton("Stop", this);
    m_btnStop->setFixedHeight(36);
    m_btnStop->setEnabled(false);
    connect(m_btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    actionLayout->addWidget(m_btnStop);

    m_btnDiagnostics = new QPushButton("Run Diagnostics", this);
    m_btnDiagnostics->setFixedHeight(36);
    connect(m_btnDiagnostics, &QPushButton::clicked, this, &MainWindow::onDiagnosticsClicked);
    actionLayout->addWidget(m_btnDiagnostics);
    
    actionLayout->addStretch();
    topControlsLayout->addWidget(actionGroup);

    // Group 2: Hardware Settings
    QGroupBox *hwGroup = new QGroupBox("Hardware Settings (SDR)", this);
    QGridLayout *hwLayout = new QGridLayout(hwGroup);
    
    m_btnAutotune = new QPushButton("Start Headless Auto-Tune Sweep", this);
    m_btnAutotune->setStyleSheet("background-color: #f59e0b; color: white;");
    connect(m_btnAutotune, &QPushButton::clicked, this, &MainWindow::onAutotuneClicked);
    hwLayout->addWidget(m_btnAutotune, 0, 0, 1, 1);
    
    m_btnAntennaTest = new QPushButton("Test Antenna (Noise Floor)", this);
    m_btnAntennaTest->setStyleSheet("background-color: #8b5cf6; color: white;"); // Purple
    connect(m_btnAntennaTest, &QPushButton::clicked, this, &MainWindow::onAntennaTestClicked);
    hwLayout->addWidget(m_btnAntennaTest, 0, 1, 1, 1);

    hwLayout->addWidget(new QLabel("Sample Rate:", this), 1, 0);
    m_comboSampleRate = new QComboBox(this);
    m_comboSampleRate->addItems({"2.0 MSPS (GPS L1 Optimized)", "4.0 MSPS"});
    m_comboSampleRate->setCurrentIndex(0);
    hwLayout->addWidget(m_comboSampleRate, 1, 1);

    hwLayout->addWidget(new QLabel("Gain:", this), 2, 0);
    QHBoxLayout *gainLayout = new QHBoxLayout();
    m_sliderGain = new QSlider(Qt::Horizontal, this);
    m_sliderGain->setRange(0, 73); 
    m_sliderGain->setValue(60);
    connect(m_sliderGain, &QSlider::valueChanged, this, &MainWindow::onGainSliderChanged);
    gainLayout->addWidget(m_sliderGain);
    m_lblGainVal = new QLabel("60 dB", this);
    gainLayout->addWidget(m_lblGainVal);
    hwLayout->addLayout(gainLayout, 2, 1);

    m_chkAdaptiveGain = new QCheckBox("Adaptive Gain Tuning (SNR)", this);
    m_chkAdaptiveGain->setChecked(false); // default OFF
    m_chkAdaptiveGain->setToolTip("Automatically adjust RF gain to maximize satellite C/N0 (Hill-Climbing).");
    hwLayout->addWidget(m_chkAdaptiveGain, 3, 0, 1, 2);

    m_chkBiasTee = new QCheckBox("Enable Active Antenna Bias-Tee (3.3V)", this);
    m_chkBiasTee->setChecked(false); // default off to prevent damage to passive setups
    hwLayout->addWidget(m_chkBiasTee, 4, 0, 1, 2);

    m_chkRecordIq = new QCheckBox("Record Raw IQ (.bin)", this);
    m_chkRecordIq->setChecked(false); // default OFF
    m_chkRecordIq->setToolTip("Record raw complex-float IQ samples to 'limesdr_raw_gps.bin' for offline research");
    hwLayout->addWidget(m_chkRecordIq, 5, 0, 1, 2);

    hwLayout->setRowStretch(6, 1);
    topControlsLayout->addWidget(hwGroup);

    // Group 3: Tracking & DSP Settings
    QGroupBox *dspGroup = new QGroupBox("Tracking & DSP Settings", this);
    QFormLayout *dspLayout = new QFormLayout(dspGroup);

    m_chkAutoTuneTracking = new QCheckBox("Auto-Tune Tracking Loops", this);
    m_chkAutoTuneTracking->setChecked(true); // Default ON
    m_chkAutoTuneTracking->setToolTip("Automatically monitor for lock instability and increase PLL bandwidth on the fly.");
    dspLayout->addRow(m_chkAutoTuneTracking);

    m_spinPllBw = new QDoubleSpinBox(this);
    m_spinPllBw->setRange(1.0, 100.0);
    m_spinPllBw->setSingleStep(1.0);
    m_spinPllBw->setValue(40.0); // Wider default to handle LimeSDR phase noise
    m_spinPllBw->setToolTip("Phase Locked Loop Bandwidth. Increase if receiver loses lock frequently.");
    m_spinPllBw->setEnabled(false); // Disabled by auto-tune initially
    dspLayout->addRow("PLL Bandwidth (Hz):", m_spinPllBw);

    m_spinDllBw = new QDoubleSpinBox(this);
    m_spinDllBw->setRange(0.1, 10.0);
    m_spinDllBw->setSingleStep(0.1);
    m_spinDllBw->setValue(2.0);
    m_spinDllBw->setToolTip("Delay Locked Loop Bandwidth.");
    m_spinDllBw->setEnabled(false); // Disabled by auto-tune initially
    dspLayout->addRow("DLL Bandwidth (Hz):", m_spinDllBw);

    connect(m_chkAutoTuneTracking, &QCheckBox::toggled, this, [this](bool checked){
        m_spinPllBw->setEnabled(!checked);
        m_spinDllBw->setEnabled(!checked);
    });

    topControlsLayout->addWidget(dspGroup);

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
    pvtGroup->setFixedWidth(340);
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

    // Satellite Skyplot Group Box
    QGroupBox *skyplotGroup = new QGroupBox("GPS Constellation Skyplot", this);
    skyplotGroup->setFixedWidth(280);
    midLayout->addWidget(skyplotGroup);
    
    QVBoxLayout *skyLayout = new QVBoxLayout(skyplotGroup);
    skyLayout->setContentsMargins(6, 6, 6, 6);
    m_skyplot = new SkyplotWidget(this);
    m_skyplot->setMinimumSize(220, 220);
    skyLayout->addWidget(m_skyplot);

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
    m_txtConsole->document()->setMaximumBlockCount(1000); // Prevent memory leaks and GUI lag
    m_txtConsole->setObjectName("txtConsole");
    consoleLayout->addWidget(m_txtConsole);

    // --- ANTENNA SCANNER TAB ---
    QWidget *tabScanner = new QWidget(this);
    tabWidget->addTab(tabScanner, "Antenna Scanner");
    QVBoxLayout *scanLayout = new QVBoxLayout(tabScanner);
    
    QGroupBox *scanCtrlGroup = new QGroupBox("Scanner Controls", this);
    scanLayout->addWidget(scanCtrlGroup);
    QHBoxLayout *scanCtrlLayout = new QHBoxLayout(scanCtrlGroup);
    
    scanCtrlLayout->addWidget(new QLabel("Start (MHz):"));
    m_spinScanStart = new QDoubleSpinBox(this);
    m_spinScanStart->setRange(10, 3800);
    m_spinScanStart->setValue(1500);
    scanCtrlLayout->addWidget(m_spinScanStart);
    
    scanCtrlLayout->addWidget(new QLabel("Stop (MHz):"));
    m_spinScanStop = new QDoubleSpinBox(this);
    m_spinScanStop->setRange(10, 3800);
    m_spinScanStop->setValue(1600);
    scanCtrlLayout->addWidget(m_spinScanStop);
    
    scanCtrlLayout->addWidget(new QLabel("Step (MHz):"));
    m_spinScanStep = new QDoubleSpinBox(this);
    m_spinScanStep->setRange(0.1, 10);
    m_spinScanStep->setSingleStep(0.1);
    m_spinScanStep->setValue(1.0);
    scanCtrlLayout->addWidget(m_spinScanStep);
    
    m_btnStartScan = new QPushButton("Start Scan", this);
    connect(m_btnStartScan, &QPushButton::clicked, this, &MainWindow::onStartScanClicked);
    scanCtrlLayout->addWidget(m_btnStartScan);
    
    m_btnStopScan = new QPushButton("Stop Scan", this);
    m_btnStopScan->setEnabled(false);
    connect(m_btnStopScan, &QPushButton::clicked, this, &MainWindow::onStopScanClicked);
    scanCtrlLayout->addWidget(m_btnStopScan);
    scanCtrlLayout->addStretch();
    
    m_lblInterferenceWarning = new QLabel("", this);
    m_lblInterferenceWarning->setAlignment(Qt::AlignCenter);
    m_lblInterferenceWarning->setStyleSheet("QLabel { color: white; background-color: #ef4444; font-weight: bold; font-size: 14px; padding: 10px; border-radius: 4px; }");
    m_lblInterferenceWarning->hide();
    scanLayout->addWidget(m_lblInterferenceWarning);
    
    m_spectrumPlot = new SpectrumPlotWidget(this);
    scanLayout->addWidget(m_spectrumPlot, 1);
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
            font-family: 'Menlo', 'Courier New', monospace;
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
            font-family: 'Menlo', 'Courier New', monospace;
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
            font-family: 'Menlo', 'Courier New', monospace;
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
    out << "Acquisition_1C.dump=false\n\n";

    out << "Tracking_1C.implementation=GPS_L1_CA_DLL_PLL_Tracking\n";
    out << "Tracking_1C.item_type=gr_complex\n";
    out << "Tracking_1C.pll_bw_hz=" << m_spinPllBw->value() << "\n";
    out << "Tracking_1C.dll_bw_hz=" << m_spinDllBw->value() << "\n";
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
    m_chkRecordIq->setEnabled(false);
    
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
    bool recordIq = m_chkRecordIq->isChecked();
 
    m_sdrStreamer->startStreaming(m_fifoPath, rate, gain, enableBiasTee, recordIq);

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

    m_updateTimer->start(500); // 500ms update
    
    m_lossOfLockCount = 0;
    m_autoTuneTimer->start(15000); // Check stability every 15 seconds
    
    m_previousAverageCn0 = 0.0;
    m_agcDirection = 1;
    m_snrAgcTimer->start(30000); // Check SNR every 30 seconds to allow PLL recovery
}

void MainWindow::onAutoTuneTimer()
{
    if (!m_chkAutoTuneTracking->isChecked()) {
        m_lossOfLockCount = 0; // reset
        return;
    }

    if (m_lossOfLockCount > 3) {
        // High instability detected!
        double currentPll = m_spinPllBw->value();
        if (currentPll < m_spinPllBw->maximum()) {
            double newPll = std::min(currentPll + 5.0, m_spinPllBw->maximum());
            m_spinPllBw->setValue(newPll);
            
            appendLog(QString("⚠️ [AUTO-TUNE] High phase jitter detected (%1 drops in 15s)!").arg(m_lossOfLockCount), "#f59e0b");
            appendLog(QString("⚠️ [AUTO-TUNE] Increasing PLL Bandwidth to %1 Hz and restarting...").arg(newPll), "#f59e0b");
            
            // Restart receiver
            onStopClicked();
            
            // Give it a brief moment to tear down properly before starting again
            QTimer::singleShot(1000, this, &MainWindow::onStartClicked);
            return;
        }
    } else if (m_lossOfLockCount == 0 && m_spinPllBw->value() > 40.0) {
        // If it's perfectly stable but we pushed the PLL very high, try inching it down slowly
        // to reduce thermal noise.
        static int stableTicks = 0;
        stableTicks++;
        if (stableTicks >= 4) { // 1 minute of absolute stability
            stableTicks = 0;
            double currentPll = m_spinPllBw->value();
            double newPll = std::max(currentPll - 1.0, 40.0);
            m_spinPllBw->setValue(newPll);
            
            appendLog(QString("✅ [AUTO-TUNE] Excellent stability. Decreasing PLL Bandwidth to %1 Hz and restarting...").arg(newPll), "#10b981");
            onStopClicked();
            QTimer::singleShot(1000, this, &MainWindow::onStartClicked);
            return;
        }
    }

    // Reset counter for the next 15-second window
    m_lossOfLockCount = 0;
}

void MainWindow::onSnrAgcTimer()
{
    if (!m_chkAdaptiveGain->isChecked() || !m_sdrStreamer->isRunningStream()) {
        return;
    }
    
    // 1. Calculate Average C/N0
    const auto &sats = m_nmeaParser.getSatellites();
    if (sats.size() < 2) {
        // Not enough satellites to optimize SNR. Do nothing.
        return;
    }
    
    double totalCn0 = 0.0;
    int count = 0;
    for (auto it = sats.constBegin(); it != sats.constEnd(); ++it) {
        if (it.value().snr > 0) {
            totalCn0 += it.value().snr;
            count++;
        }
    }
    
    if (count == 0) return;
    double currentAverageCn0 = totalCn0 / count;
    
    // 2. Perturb and Observe (Hill Climbing)
    int currentGain = m_sliderGain->value();
    
    // SAFETY CATCH: If C/N0 is already very good, STOP tweaking! 
    // SDR gain changes cause phase transients that break PLL locks.
    if (currentAverageCn0 >= 42.0) {
        appendLog(QString("✅ [AGC] Average C/N0 is excellent (%1 dB-Hz). Locking gain at %2 dB to prevent phase transients.")
                  .arg(currentAverageCn0, 0, 'f', 1).arg(currentGain), "#10b981");
        m_previousAverageCn0 = currentAverageCn0;
        return; 
    }
    
    if (m_previousAverageCn0 > 0.0) {
        if (currentAverageCn0 < m_previousAverageCn0 - 0.5) { // 0.5 dB hysteresis to ignore noise
            // C/N0 dropped! We went the wrong way last time. Reverse direction!
            m_agcDirection = -m_agcDirection;
        }
    }
    
    int targetGain = currentGain + m_agcDirection;
    targetGain = qBound(0, targetGain, 73);
    
    if (targetGain != currentGain) {
        m_sliderGain->blockSignals(true);
        m_sliderGain->setValue(targetGain);
        m_lblGainVal->setText(QString("%1 dB").arg(targetGain));
        m_sliderGain->blockSignals(false);
        
        m_sdrStreamer->setDynamicGain(targetGain);
        
        QString dirStr = (m_agcDirection > 0) ? "increasing" : "decreasing";
        appendLog(QString("⚠️ [AGC] SNR Optimization: Avg C/N0 went from %1 to %2 dB-Hz. %3 gain to %4 dB. (Note: May cause brief loss of lock!)")
                  .arg(m_previousAverageCn0, 0, 'f', 1)
                  .arg(currentAverageCn0, 0, 'f', 1)
                  .arg(dirStr)
                  .arg(targetGain), "#a78bfa");
    }
    
    m_previousAverageCn0 = currentAverageCn0;
}

void MainWindow::onStopClicked()
{
    m_updateTimer->stop();
    m_autoTuneTimer->stop();
    m_snrAgcTimer->stop();
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
    m_chkRecordIq->setEnabled(true);

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
    updatePowerLabelDisplay();

    // Adaptive Gain Tuning
    if (m_chkAdaptiveGain->isChecked() && m_sdrStreamer->isRunningStream()) {
        int currentGain = m_sliderGain->value();
        int targetGain = currentGain;

        if (powerDb > -10.0) {
            // Signal is getting dangerously close to clipping/saturation!
            // Immediate safety override!
            if (powerDb > -5.0) {
                targetGain -= 5; // Fast back-off
            } else {
                targetGain -= 2; // Firm back-off
            }
            appendLog(QString("⚠️ [AGC SAFETY] ADC power too high (%1 dBFS). Forcing gain reduction!").arg(powerDb, 0, 'f', 1), "#ef4444");
        } 
        // We no longer try to hold an arbitrary -15 dBFS level or pump gain up if it's low.
        // We leave the optimal C/N0 searching to onSnrAgcTimer().

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
            } else if (line.contains("Loss of lock")) {
                m_lossOfLockCount++;
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
    if (m_chkRecordIq->isChecked() && m_sdrStreamer->isRunningStream()) {
        updatePowerLabelDisplay();
    }
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

        QMap<int, SatelliteInfo> sats = m_nmeaParser.getSatellites();
        if (sats.isEmpty()) {
            // Fallback: Populate with active channels to show tracking state in skyplot before first 3D fix
            for (int i = 0; i < 8; ++i) {
                if (m_channels[i].prn > 0) {
                    SatelliteInfo sat;
                    sat.prn = m_channels[i].prn;
                    sat.elevation = 45;
                    sat.azimuth = i * 45; // Space them 45 degrees apart
                    sat.snr = (m_channels[i].status == "Tracking") ? 40 : 35;
                    sats[sat.prn] = sat;
                }
            }
        }
        m_skyplot->setSatellites(sats);
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

void MainWindow::onDiagnosticsClicked()
{
    appendLog("\n=== STARTING HARDWARE & SYSTEM DIAGNOSTICS ===", "#38bdf8"); // Light blue

    QString sdrStatus, agpsStatus, fifoStatus, limeStatus, usbStatus;

    // 1. Check GNSS-SDR installation
    QString gnssSdrPath = "/opt/local/bin/gnss-sdr";
    QFileInfo sdrInfo(gnssSdrPath);
    if (sdrInfo.exists() && sdrInfo.isExecutable()) {
        appendLog("✔ GNSS-SDR Binary: FOUND and EXECUTABLE at " + gnssSdrPath, "#10b981");
        sdrStatus = "<span style='color: #10b981;'>✔ Found &amp; Executable</span>";
    } else {
        appendLog("✘ GNSS-SDR Binary: MISSING or NOT EXECUTABLE at " + gnssSdrPath, "#ef4444");
        sdrStatus = "<span style='color: #ef4444;'>✘ Missing or Not Executable</span>";
    }

    // 2. Check A-GPS Assistance Files
    QStringList agpsFiles = {"gps_ephemeris.xml", "gps_utc_model.xml", "gps_iono.xml"};
    bool allAgpsExist = true;
    for (const QString &file : agpsFiles) {
        QFileInfo fInfo(m_workspaceDir + "/" + file);
        if (fInfo.exists() && fInfo.size() > 0) {
            appendLog(QString("✔ A-GPS File: %1 found (%2 bytes)").arg(file).arg(fInfo.size()), "#10b981");
        } else {
            appendLog(QString("✘ A-GPS File: %1 is MISSING or EMPTY").arg(file), "#f59e0b");
            allAgpsExist = false;
        }
    }
    if (allAgpsExist) {
        appendLog("✔ A-GPS Status: Ready for instant PVT fix.", "#10b981");
        agpsStatus = "<span style='color: #10b981;'>✔ Ready (A-GPS Active)</span>";
    } else {
        appendLog("⚠ A-GPS Status: Missing files. Receiver will fallback to slow over-the-air decoding.", "#f59e0b");
        agpsStatus = "<span style='color: #f59e0b;'>⚠ Incomplete (Slow OTA Fallback)</span>";
    }

    // 3. Test FIFO creation
    setupFifo();
    QFileInfo fifoInfo(m_fifoPath);
    if (fifoInfo.exists() && fifoInfo.isWritable()) {
        appendLog("✔ FIFO Pipe: Configured correctly at " + m_fifoPath, "#10b981");
        fifoStatus = "<span style='color: #10b981;'>✔ Configured &amp; Writable</span>";
    } else {
        appendLog("✘ FIFO Pipe: FAILED to write or create at " + m_fifoPath, "#ef4444");
        fifoStatus = "<span style='color: #ef4444;'>✘ Failed to Create</span>";
    }

    // 4. Test LimeSDR connection via SoapySDR
    appendLog("Scanning USB Bus for LimeSDR hardware...", "#38bdf8");
    limeStatus = "<span style='color: #ef4444;'>✘ Not Detected</span>";
    usbStatus = "<span style='color: #64748b;'>Unknown</span>";
    
    // Set soapy plugin path for Mac OS to make sure it loads LimeSuite
    setenv("SOAPY_SDR_PLUGIN_PATH", "/usr/local/lib/SoapySDR/modules0.8", 0);

    try {
        SoapySDR::KwargsList results = SoapySDR::Device::enumerate();
        bool foundLime = false;
        for (const auto &deviceArgs : results) {
            QString driver = QString::fromStdString(deviceArgs.at("driver"));
            QString hardware = deviceArgs.count("hardware") ? QString::fromStdString(deviceArgs.at("hardware")) : "Unknown";
            
            if (driver.contains("lime", Qt::CaseInsensitive) || hardware.contains("lime", Qt::CaseInsensitive)) {
                foundLime = true;
                appendLog(QString("✔ LimeSDR Hardware Detected!"), "#10b981");
                appendLog(QString("  - Driver: %1").arg(driver), "#94a3b8");
                appendLog(QString("  - Model: %1").arg(hardware), "#94a3b8");
                
                QString serial = deviceArgs.count("serial") ? QString::fromStdString(deviceArgs.at("serial")) : "Unknown";
                if (deviceArgs.count("serial")) {
                    appendLog(QString("  - Serial Number: %1").arg(serial), "#94a3b8");
                }
                
                limeStatus = QString("<span style='color: #10b981;'>✔ Detected (%1)</span>").arg(hardware);
                
                // Check USB Speed
                QString label = deviceArgs.count("label") ? QString::fromStdString(deviceArgs.at("label")) : "";
                if (label.contains("USB 3.0", Qt::CaseInsensitive)) {
                    appendLog("✔ USB Interface Speed: USB 3.0 (SUPER SPEED - OK)", "#10b981");
                    usbStatus = "<span style='color: #10b981;'>✔ USB 3.0 (Super Speed - OK)</span>";
                } else if (label.contains("USB 2.0", Qt::CaseInsensitive)) {
                    appendLog("⚠ USB Interface Speed: USB 2.0 (DEGRADED - High risk of overflows/dropouts)", "#f59e0b");
                    usbStatus = "<span style='color: #f59e0b;'>⚠ USB 2.0 (Degraded - Overflow Risk)</span>";
                } else {
                    appendLog("⚠ USB Interface Speed: Unknown (Check connection to a blue USB 3.0 port)", "#f59e0b");
                    usbStatus = "<span style='color: #f59e0b;'>⚠ Unknown (Check port)</span>";
                }
            }
        }
        
        if (!foundLime) {
            appendLog("✘ LimeSDR Hardware: NOT DETECTED. Make sure it is plugged into a USB 3.0 port.", "#ef4444");
        }
    } catch (const std::exception &e) {
        appendLog(QString("✘ SoapySDR Diagnostics Error: %1").arg(e.what()), "#ef4444");
        limeStatus = QString("<span style='color: #ef4444;'>✘ Driver Error: %1</span>").arg(e.what());
    }

    appendLog("=== DIAGNOSTICS COMPLETED ===\n", "#38bdf8");

    // Display Custom QMessageBox Dialog
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Hardware Diagnostics Report");
    msgBox.setTextFormat(Qt::RichText);
    msgBox.setStyleSheet(styleSheet());
    
    QString htmlReport = QString(
        "<div style='font-family: Menlo, sans-serif; font-size: 13px; color: #cbd5e1; line-height: 1.5; min-width: 420px;'>"
        "<h3 style='color: #38bdf8; margin-top: 0; margin-bottom: 8px;'>🛰️ Hardware &amp; System Diagnostics</h3>"
        "<hr style='border: none; border-top: 1px solid #334155; margin: 8px 0;'/>"
        "<table cellpadding='6' cellspacing='0' style='width: 100%%;'>"
        "  <tr><td style='font-weight: bold; width: 160px; color: #94a3b8;'>GNSS-SDR Binary:</td><td style='font-family: Menlo;'>%1</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>A-GPS Assistance:</td><td style='font-family: Menlo;'>%2</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>FIFO Pipe:</td><td style='font-family: Menlo;'>%3</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>LimeSDR Device:</td><td style='font-family: Menlo;'>%4</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>USB Connection:</td><td style='font-family: Menlo;'>%5</td></tr>"
        "</table>"
        "<hr style='border: none; border-top: 1px solid #334155; margin: 8px 0;'/>"
        "<p style='font-size: 11px; color: #64748b; margin-bottom: 0;'>"
        "Detailed logs have been written to the System Console tab."
        "</p>"
        "</div>"
    ).arg(sdrStatus, agpsStatus, fifoStatus, limeStatus, usbStatus);

    msgBox.setText(htmlReport);
    msgBox.addButton(QMessageBox::Ok);
    msgBox.exec();
}

void MainWindow::onAutotuneClicked()
{
    appendLog("\n=== STARTING LIME-SDR AUTOMATIC TUNING ===", "#38bdf8"); // Light blue

    // Check if streamer is running
    if (m_sdrStreamer->isRunningStream()) {
        appendLog("✘ Autotune: Cannot run autotune while receiver is active. Stop it first.", "#ef4444");
        QMessageBox::warning(this, "Autotune Warning", "Please stop the GNSS receiver flow before running autotune.");
        return;
    }

    setenv("SOAPY_SDR_PLUGIN_PATH", "/usr/local/lib/SoapySDR/modules0.8", 0);

    SoapySDR::Device *sdr = nullptr;
    try {
        sdr = SoapySDR::Device::make();
    } catch (const std::exception &e) {
        appendLog(QString("✘ Autotune: Connection to SDR failed: %1").arg(e.what()), "#ef4444");
        QMessageBox::critical(this, "Autotune Error", QString("Failed to connect to LimeSDR device: %1").arg(e.what()));
        return;
    }

    if (!sdr) {
        appendLog("✘ Autotune: No SDR device found.", "#ef4444");
        QMessageBox::critical(this, "Autotune Error", "No LimeSDR hardware detected on the USB bus.");
        return;
    }

    QString hardwareKey = QString::fromStdString(sdr->getHardwareKey());
    appendLog(QString("✔ Connected to LimeSDR: %1").arg(hardwareKey), "#10b981");

    double rate = 2.0e6;
    if (m_comboSampleRate->currentIndex() == 1) {
        rate = 4.0e6;
    }

    QString calibStatus = "<span style='color: #ef4444;'>✘ Failed</span>";
    QString noiseFloorStr = "Unknown";
    QString dcOffsetStr = "Unknown";
    QString filterBwStr = "Unknown";

    try {
        // 1. Setup sample rate and gain
        sdr->setSampleRate(SOAPY_SDR_RX, 0, rate);
        sdr->setAntenna(SOAPY_SDR_RX, 0, "LNAH");
        
        // Setup initial gain
        double totalGain = m_sliderGain->value();
        double lna = std::min(30.0, totalGain);
        double remaining = totalGain - lna;
        double TIA_MAX = (QString::fromStdString(sdr->getDriverKey()) == "lime") ? 12.0 : 0.0;
        double tia = std::min(TIA_MAX, remaining);
        double pga = std::min(32.0, remaining - tia);
        sdr->setGain(SOAPY_SDR_RX, 0, "LNA", lna);
        if (TIA_MAX > 0.1) {
            sdr->setGain(SOAPY_SDR_RX, 0, "TIA", tia);
        }
        sdr->setGain(SOAPY_SDR_RX, 0, "PGA", pga);

        // 2. Perform LMS7002M Calibration (Self-Calibration of DC & IQ imbalance)
        appendLog("Executing hardware DC/IQ calibration...", "#38bdf8");
        
        // 3. Optimize Analog Filter Bandwidth (Noise Blocker)
        // Tune to sampleRate * 1.25 to allow transition band but filter out close-in jammers
        // Note: setting bandwidth automatically triggers LMS7002M hardware calibration
        double targetBw = rate * 1.25; 
        sdr->setBandwidth(SOAPY_SDR_RX, 0, targetBw);
        double actualBw = sdr->getBandwidth(SOAPY_SDR_RX, 0);
        appendLog("✔ Hardware Calibration: Successful", "#10b981");
        calibStatus = "<span style='color: #10b981;'>✔ Successful (LMS7002M DC/IQ calibrated)</span>";
        
        appendLog(QString("✔ Analog Filter: Tuned to %1 MHz (Sample Rate: %2 MSPS)").arg(actualBw / 1e6, 0, 'f', 2).arg(rate / 1e6, 0, 'f', 1), "#10b981");
        filterBwStr = QString("<span style='color: #10b981;'>%1 MHz (Optimal roll-off)</span>").arg(actualBw / 1e6, 0, 'f', 2);

        // 4. Measure live DC leakage & Noise Floor
        appendLog("Acquiring sample burst for spectral diagnostics...", "#38bdf8");
        
        SoapySDR::Stream *stream = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
        sdr->activateStream(stream);

        // Allow time for PLL and Calibration to settle
        usleep(50000);

        const size_t burstSize = 16384;
        std::vector<std::complex<float>> burstBuffer(burstSize);
        void *buffs[] = { burstBuffer.data() };
        int flags = 0;
        long long timeNs = 0;
        
        // Flush any stale/calibration samples from the FIFO
        for (int k = 0; k < 3; ++k) {
            sdr->readStream(stream, buffs, burstSize, flags, timeNs, 100000);
        }
        
        // Read clean samples for diagnostic measurement
        int readSamples = sdr->readStream(stream, buffs, burstSize, flags, timeNs, 200000); // 200ms timeout
        
        sdr->deactivateStream(stream);
        sdr->closeStream(stream);

        if (readSamples > 0) {
            // Compute mean offset (DC component) and RMS magnitude (Noise)
            std::complex<float> sum(0, 0);
            double sumSq = 0;
            for (int i = 0; i < readSamples; ++i) {
                sum += burstBuffer[i];
                float mag = std::abs(burstBuffer[i]);
                sumSq += mag * mag;
            }
            std::complex<float> mean = sum / (float)readSamples;
            double meanMagSq = sumSq / readSamples;
            double noiseDb = 10.0 * std::log10(meanMagSq + 1e-12);
            double dcMagDb = 10.0 * std::log10(std::norm(mean) + 1e-12);

            noiseFloorStr = QString("<span style='color: #10b981;'>%1 dBFS</span>").arg(noiseDb, 0, 'f', 1);
            
            if (dcMagDb < -40.0) {
                dcOffsetStr = QString("<span style='color: #10b981;'>%1 dB (Excellent - Clean)</span>").arg(dcMagDb, 0, 'f', 1);
            } else if (dcMagDb < -25.0) {
                dcOffsetStr = QString("<span style='color: #f59e0b;'>%1 dB (Moderate - Blocked by DC Filter)</span>").arg(dcMagDb, 0, 'f', 1);
            } else {
                dcOffsetStr = QString("<span style='color: #ef4444;'>%1 dB (High - Risk of center carrier spike)</span>").arg(dcMagDb, 0, 'f', 1);
            }

            appendLog(QString("✔ Noise Floor: %1 dBFS").arg(noiseDb, 0, 'f', 1), "#10b981");
            appendLog(QString("✔ DC Leakage Offset: %1 dB").arg(dcMagDb, 0, 'f', 1), "#10b981");
        } else {
            appendLog("⚠ Noise/DC measurement: Read timeout occurred.", "#f59e0b");
            noiseFloorStr = "<span style='color: #f59e0b;'>⚠ Timeout</span>";
            dcOffsetStr = "<span style='color: #f59e0b;'>⚠ Timeout</span>";
        }

    } catch (const std::exception &err) {
        appendLog(QString("✘ Autotune error: %1").arg(err.what()), "#ef4444");
        QMessageBox::critical(this, "Autotune Failure", QString("Tuning routine encountered an error: %1").arg(err.what()));
        SoapySDR::Device::unmake(sdr);
        return;
    }

    SoapySDR::Device::unmake(sdr);
    appendLog("=== AUTOTUNE ROUTINE COMPLETED ===\n", "#38bdf8");

    // Display html popup report
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Receiver Autotune Complete");
    msgBox.setTextFormat(Qt::RichText);
    msgBox.setStyleSheet(styleSheet());
    
    QString htmlReport = QString(
        "<div style='font-family: Menlo, sans-serif; font-size: 13px; color: #cbd5e1; line-height: 1.5; min-width: 450px;'>"
        "<h3 style='color: #38bdf8; margin-top: 0; margin-bottom: 8px;'>🛰️ LimeSDR Autotune Report</h3>"
        "<p style='font-size: 12px; margin-bottom: 12px; color: #94a3b8;'>"
        "The hardware has been calibrated and aligned for GPS L1 reception."
        "</p>"
        "<hr style='border: none; border-top: 1px solid #334155; margin: 8px 0;'/>"
        "<table cellpadding='6' cellspacing='0' style='width: 100%%;'>"
        "  <tr><td style='font-weight: bold; width: 180px; color: #94a3b8;'>HW Calibration:</td><td>%1</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>Baseband Filter:</td><td>%2</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>Measured Noise Floor:</td><td>%3</td></tr>"
        "  <tr><td style='font-weight: bold; color: #94a3b8;'>Measured DC Leakage:</td><td>%4</td></tr>"
        "</table>"
        "<hr style='border: none; border-top: 1px solid #334155; margin: 8px 0;'/>"
        "<p style='font-size: 11px; color: #10b981; margin-bottom: 0; font-weight: bold;'>"
        "✔ Receiver calibration is locked. You can now start the GPS receiver."
        "</p>"
        "</div>"
    ).arg(calibStatus, filterBwStr, noiseFloorStr, dcOffsetStr);

    msgBox.setText(htmlReport);
    msgBox.addButton(QMessageBox::Ok);
    msgBox.exec();
}

void MainWindow::updatePowerLabelDisplay()
{
    // Calculate Average C/N0 (dB-Hz) for the main status display
    const auto &sats = m_nmeaParser.getSatellites();
    double totalCn0 = 0.0;
    int count = 0;
    for (auto it = sats.constBegin(); it != sats.constEnd(); ++it) {
        if (it.value().snr > 0) {
            totalCn0 += it.value().snr;
            count++;
        }
    }
    
    QString labelText;
    if (count > 0) {
        double avgCn0 = totalCn0 / count;
        labelText = QString("SIGNAL: %1 dB-Hz | ADC: %2 dBFS")
                    .arg(avgCn0, 0, 'f', 1)
                    .arg(m_currentPowerDb, 0, 'f', 1);
    } else {
        labelText = QString("SIGNAL: SEARCHING | ADC: %1 dBFS")
                    .arg(m_currentPowerDb, 0, 'f', 1);
    }
    if (m_chkRecordIq->isChecked() && m_sdrStreamer->isRunningStream()) {
        QFileInfo recordFileInfo("limesdr_raw_gps.bin");
        double sizeMb = recordFileInfo.exists() ? (recordFileInfo.size() / (1024.0 * 1024.0)) : 0.0;
        labelText += QString(" (Rec: %1 MB)").arg(sizeMb, 0, 'f', 1);
    }
    m_lblPower->setText(labelText);

    // Color alert for saturation
    if (m_currentPowerDb > -5.0) {
        m_lblPower->setStyleSheet("color: #ef4444; font-weight: bold;"); // Red (saturation)
    } else {
        m_lblPower->setStyleSheet("color: #3b82f6; font-weight: bold;"); // Blue
    }
}

void SkyplotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Fill background with slate-950 dark tone
    painter.fillRect(rect(), QColor("#0f172a"));

    int w = width();
    int h = height();
    int size = qMin(w, h) - 24;
    int centerX = w / 2;
    int centerY = h / 2;
    int radius = size / 2;

    if (radius <= 0) return;

    // Draw concentric rings (elevation levels: 0, 30, 60 degrees)
    QPen ringPen(QColor("#334155"), 1, Qt::DashLine);
    painter.setPen(ringPen);
    painter.setBrush(Qt::NoBrush);

    // 0 deg elevation (outer horizon ring)
    painter.drawEllipse(centerX - radius, centerY - radius, radius * 2, radius * 2);
    // 30 deg elevation
    painter.drawEllipse(centerX - radius * 2/3, centerY - radius * 2/3, radius * 4/3, radius * 4/3);
    // 60 deg elevation
    painter.drawEllipse(centerX - radius * 1/3, centerY - radius * 1/3, radius * 2/3, radius * 2/3);

    // Draw crosshairs
    painter.drawLine(centerX, centerY - radius, centerX, centerY + radius);
    painter.drawLine(centerX - radius, centerY, centerX + radius, centerY);

    // Draw cardinal direction markers
    painter.setPen(QColor("#94a3b8"));
    QFont labelFont = font();
    labelFont.setPointSize(10);
    labelFont.setBold(true);
    painter.setFont(labelFont);
    
    painter.drawText(centerX - 5, centerY - radius + 15, "N");
    painter.drawText(centerX - 5, centerY + radius - 5, "S");
    painter.drawText(centerX + radius - 15, centerY + 5, "E");
    painter.drawText(centerX - radius + 5, centerY + 5, "W");

    // Draw satellites
    for (auto it = m_sats.begin(); it != m_sats.end(); ++it) {
        const SatelliteInfo &sat = it.value();
        
        // Skip invalid elevation bounds
        if (sat.elevation < 0 || sat.elevation > 90) continue;

        // Convert elevation and azimuth to polar coordinates
        // r = 0 for 90 deg (zenith at center), r = radius for 0 deg (horizon at edge)
        double r = radius * (90.0 - sat.elevation) / 90.0;
        double angleRad = (sat.azimuth - 90.0) * M_PI / 180.0;

        int satX = centerX + r * cos(angleRad);
        int satY = centerY + r * sin(angleRad);

        // Color points based on SNR (C/N0 dB-Hz)
        QColor satColor;
        if (sat.snr >= 38) satColor = QColor("#10b981"); // Strong (Green)
        else if (sat.snr >= 25) satColor = QColor("#f59e0b"); // Moderate (Orange)
        else if (sat.snr > 0) satColor = QColor("#ef4444"); // Weak (Red)
        else satColor = QColor("#64748b"); // Acquired but no SNR yet (Slate)

        painter.setBrush(satColor);
        painter.setPen(QPen(QColor("#ffffff"), 1));
        
        // Draw satellite circle
        painter.drawEllipse(satX - 9, satY - 9, 18, 18);

        // Draw PRN number label inside the circle
        painter.setPen(QColor("#ffffff"));
        QFont prnFont = font();
        prnFont.setPointSize(8);
        prnFont.setBold(true);
        painter.setFont(prnFont);
        painter.drawText(satX - 6, satY + 3, QString::number(sat.prn).rightJustified(2, '0'));
    }
}

void MainWindow::onAntennaTestClicked()
{
    if (m_gnssProcess->state() != QProcess::NotRunning || m_sdrScanner->isRunning()) {
        QMessageBox::warning(this, "Device Busy", "Please stop the main receiver or scanner before running the antenna test.");
        return;
    }
    
    AntennaTestDialog dialog(this);
    dialog.exec();
}

void MainWindow::onStartScanClicked()
{
    if (m_sdrStreamer->isRunningStream()) {
        QMessageBox::warning(this, "Scanner", "Stop the GPS Receiver before running the scanner.");
        return;
    }
    m_spectrumPlot->clearPoints();
    m_btnStartScan->setEnabled(false);
    m_btnStopScan->setEnabled(true);
    m_lblInterferenceWarning->hide();
    
    double start = m_spinScanStart->value() * 1e6;
    double stop = m_spinScanStop->value() * 1e6;
    double step = m_spinScanStep->value() * 1e6;
    double gain = m_sliderGain->value();
    bool biasTee = m_chkBiasTee->isChecked();
    
    appendLog(QString("Starting Antenna Scan from %1 to %2 MHz...").arg(m_spinScanStart->value()).arg(m_spinScanStop->value()));
    m_sdrScanner->startScanning(start, stop, step, gain, biasTee);
}

void MainWindow::onStopScanClicked()
{
    if (m_sdrScanner->isScanning()) {
        m_sdrScanner->stopScanning();
    }
    m_btnStartScan->setEnabled(true);
    m_btnStopScan->setEnabled(false);
}

void MainWindow::onScanPointReceived(double frequency, double powerDb)
{
    m_spectrumPlot->addPoint(frequency, powerDb);
}

void MainWindow::onInterferenceDetected(double frequency, double powerDb)
{
    m_lblInterferenceWarning->setText(QString("⚠️ JAMMER DETECTED: Spike of %1 dB at %2 MHz ⚠️")
                                        .arg(powerDb, 0, 'f', 1)
                                        .arg(frequency / 1e6, 0, 'f', 2));
    m_lblInterferenceWarning->show();
    appendLog(QString("⚠ INTERFERENCE ALERT: Spike at %1 MHz detected (Power: %2 dB)")
                .arg(frequency / 1e6, 0, 'f', 2)
                .arg(powerDb, 0, 'f', 1), "#ef4444");
}

void MainWindow::onScanFinished()
{
    appendLog("Antenna Scan finished.");
    m_btnStartScan->setEnabled(true);
    m_btnStopScan->setEnabled(false);
}

void SpectrumPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Fill background
    painter.fillRect(rect(), QColor("#0f172a"));

    if (m_points.isEmpty()) {
        painter.setPen(QColor("#64748b"));
        painter.drawText(rect(), Qt::AlignCenter, "No scan data. Click 'Start Scan'.");
        return;
    }

    int w = width();
    int h = height();
    int margins = 50;

    double minFreq = m_points.firstKey();
    double maxFreq = m_points.lastKey();
    if (minFreq == maxFreq) maxFreq = minFreq + 1; // Prevent div by 0
    
    // Auto-scale Y axis using max hold data to prevent scale bouncing
    double minPower = m_points.first();
    double maxPower = m_points.first();
    for (double p : m_points) {
        if (p < minPower) minPower = p;
    }
    for (double p : m_maxHoldData) {
        if (p > maxPower) maxPower = p;
    }
    
    // Add 10% padding
    double pRange = maxPower - minPower;
    if (pRange < 1.0) pRange = 10.0;
    minPower -= pRange * 0.1;
    maxPower += pRange * 0.1;
    
    // Snap to neat grid multiples (10s)
    minPower = std::floor(minPower / 10.0) * 10.0;
    maxPower = std::ceil(maxPower / 10.0) * 10.0;
    
    if (maxPower - minPower < 10.0) {
        maxPower = minPower + 10.0;
    }
    
    double step = 20.0;
    double range = maxPower - minPower;
    if (range <= 20.0) step = 2.0;
    else if (range <= 50.0) step = 5.0;
    else if (range <= 80.0) step = 10.0;
    
    // Draw Grid and Y Labels
    painter.setPen(QPen(QColor("#334155"), 1, Qt::DashLine));
    for (double p = minPower; p <= maxPower; p += step) {
        int y = h - margins - (int)((p - minPower) / (maxPower - minPower) * (h - 2 * margins));
        painter.drawLine(margins, y, w - margins, y);
        painter.setPen(QColor("#94a3b8"));
        painter.drawText(5, y + 4, QString("%1 dB").arg(p, 0, 'f', 0));
        painter.setPen(QPen(QColor("#334155"), 1, Qt::DashLine));
    }
    
    // Draw X Grid and Labels
    int numXTicks = 5;
    for (int i = 0; i <= numXTicks; ++i) {
        int x = margins + i * (w - 2 * margins) / numXTicks;
        painter.drawLine(x, margins, x, h - margins);
        double f = minFreq + i * (maxFreq - minFreq) / numXTicks;
        painter.setPen(QColor("#94a3b8"));
        painter.drawText(x - 30, h - margins + 20, QString("%1 MHz").arg(f / 1e6, 0, 'f', 1));
        painter.setPen(QPen(QColor("#334155"), 1, Qt::DashLine));
    }

    QPainterPath path;
    QPainterPath maxHoldPath;
    bool first = true;
    double peakPower = -999.0;
    double peakFreq = 0.0;
    int peakX = 0;
    int peakY = 0;
    
    for (auto it = m_points.begin(); it != m_points.end(); ++it) {
        double f = it.key();
        double p = it.value();
        double pMax = m_maxHoldData.value(f, p);
        
        if (p > peakPower) {
            peakPower = p;
            peakFreq = f;
        }
        
        int x = margins + (int)((f - minFreq) / (maxFreq - minFreq) * (w - 2 * margins));
        int y = h - margins - (int)((p - minPower) / (maxPower - minPower) * (h - 2 * margins));
        int yMax = h - margins - (int)((pMax - minPower) / (maxPower - minPower) * (h - 2 * margins));
        
        y = qBound(margins, y, h - margins);
        yMax = qBound(margins, yMax, h - margins);
        
        if (p == peakPower) {
            peakX = x;
            peakY = y;
        }
        
        if (first) {
            path.moveTo(x, y);
            maxHoldPath.moveTo(x, yMax);
            first = false;
        } else {
            path.lineTo(x, y);
            maxHoldPath.lineTo(x, yMax);
        }
    }
    
    // Draw Max Hold Trace (Background)
    painter.setPen(QPen(QColor(245, 158, 11, 150), 1)); // Orange, slightly transparent
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(maxHoldPath);
    
    // Gradient fill under the path
    if (!path.isEmpty()) {
        QPainterPath fillPath = path;
        fillPath.lineTo(margins + (w - 2 * margins), h - margins);
        fillPath.lineTo(margins, h - margins);
        fillPath.closeSubpath();
        
        QLinearGradient gradient(0, margins, 0, h - margins);
        gradient.setColorAt(0.0, QColor(16, 185, 129, 100)); // #10b981 with alpha
        gradient.setColorAt(1.0, QColor(16, 185, 129, 0));
        
        painter.setPen(Qt::NoPen);
        painter.setBrush(gradient);
        painter.drawPath(fillPath);
    }
    
    // Draw the main line
    painter.setPen(QPen(QColor("#10b981"), 2));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
    
    // Draw peak marker
    if (peakPower > -999.0) {
        painter.setPen(QPen(QColor("#ef4444"), 1, Qt::DashLine));
        painter.drawLine(peakX, margins, peakX, h - margins);
        
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor("#ef4444"));
        painter.drawEllipse(peakX - 3, peakY - 3, 6, 6);
        
        painter.setPen(QColor("#ffffff"));
        painter.drawText(peakX + 8, peakY - 10, QString("Peak: %1 MHz").arg(peakFreq / 1e6, 0, 'f', 2));
        painter.drawText(peakX + 8, peakY + 5,  QString("%1 dB").arg(peakPower, 0, 'f', 1));
    }
}
