#include "headlesstuner.h"
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QRegularExpression>
#include <sys/stat.h>

HeadlessTuner::HeadlessTuner(QObject *parent) : QObject(parent), m_currentIndex(0), m_currentPvtSuccess(false)
{
    m_gnssProcess = new QProcess(this);
    m_sdrStreamer = new SdrStreamer(); // Do not set parent since it's a QThread
    m_testTimer = new QTimer(this);
    m_testTimer->setSingleShot(true);

    connect(m_testTimer, &QTimer::timeout, this, &HeadlessTuner::onTestTimeout);
    connect(m_gnssProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, &HeadlessTuner::onGnssProcessFinished);
    connect(m_gnssProcess, &QProcess::readyReadStandardOutput, this, &HeadlessTuner::onGnssProcessReadyRead);
    connect(m_gnssProcess, &QProcess::readyReadStandardError, this, &HeadlessTuner::onGnssProcessReadyRead);

    QString workspace = QDir::homePath() + "/.gnss_workspace";
    QDir().mkpath(workspace);
    m_fifoPath = workspace + "/gnss_iq.fifo";
    m_configPath = workspace + "/gnss-sdr-headless.conf";
    
    // Create CSV header
    QFile csvFile("tuning_results.csv");
    if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&csvFile);
        out << "Gain,PLL_BW,DLL_BW,PVT_Success,Avg_CN0\n";
        csvFile.close();
    }
}

void HeadlessTuner::startTuning()
{
    // Define parameter grid
    QList<int> gains = {45, 50, 55, 60, 65};
    QList<double> pllBws = {30.0, 35.0, 40.0, 45.0, 50.0};
    QList<double> dllBws = {1.0, 1.5, 2.0, 2.5, 3.0};

    for (int g : gains) {
        for (double p : pllBws) {
            for (double d : dllBws) {
                m_grid.append({g, p, d});
            }
        }
    }

    qDebug() << "Starting Headless Parameter Tuning. Total configurations:" << m_grid.size();
    m_currentIndex = 0;
    runNextConfiguration();
}

void HeadlessTuner::runNextConfiguration()
{
    if (m_currentIndex >= m_grid.size()) {
        qDebug() << "Tuning complete! Results saved to tuning_results.csv";
        if (m_sdrStreamer->isRunningStream()) {
            m_sdrStreamer->stopStreaming();
            m_sdrStreamer->wait();
        }
        delete m_sdrStreamer;
        QCoreApplication::quit();
        return;
    }

    const TuningConfig &config = m_grid[m_currentIndex];
    qDebug() << "\n=============================================";
    qDebug() << QString("Running Configuration %1/%2").arg(m_currentIndex + 1).arg(m_grid.size());
    qDebug() << QString("Gain: %1 dB | PLL BW: %2 Hz | DLL BW: %3 Hz").arg(config.gain).arg(config.pllBw).arg(config.dllBw);
    qDebug() << "=============================================";

    m_currentPvtSuccess = false;
    m_nmeaParser = NmeaParser(); // Reset parser

    // Clean old files
    QFile::remove(m_fifoPath);
    QFile::remove("gnss_sdr_pvt_limesdr.nmea");
    
    // Make FIFO
    mkfifo(m_fifoPath.toUtf8().constData(), 0666);

    // Generate config
    generateGnssConfig(config);

    // Start SDR Streamer
    if (m_sdrStreamer->isRunningStream()) {
        m_sdrStreamer->stopStreaming();
        m_sdrStreamer->wait();
    }
    
    m_sdrStreamer->startStreaming(m_fifoPath, 2000000, config.gain, true, false);

    // Start GNSS-SDR
    m_gnssProcess->start("gnss-sdr", QStringList() << "--config_file=" + m_configPath);
    
    // Wait for 120 seconds for PVT
    m_testTimer->start(120000); 
}

void HeadlessTuner::onTestTimeout()
{
    qDebug() << "Test duration complete. Terminating gnss-sdr...";
    
    if (m_gnssProcess->state() != QProcess::NotRunning) {
        m_gnssProcess->terminate();
        if (!m_gnssProcess->waitForFinished(3000)) {
            m_gnssProcess->kill();
        }
    }
}

void HeadlessTuner::onGnssProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    Q_UNUSED(exitCode);
    Q_UNUSED(exitStatus);
    
    if (m_sdrStreamer->isRunningStream()) {
        m_sdrStreamer->stopStreaming();
        m_sdrStreamer->wait();
    }
    
    // Read final NMEA to check for PVT and calculate average C/N0
    QFile nmeaFile("gnss_sdr_pvt_limesdr.nmea");
    if (nmeaFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&nmeaFile);
        while (!in.atEnd()) {
            QString line = in.readLine();
            m_nmeaParser.parseLine(line);
        }
        nmeaFile.close();
    }
    
    // Check if PVT was successful
    if (m_nmeaParser.getPosition().hasFix) {
        m_currentPvtSuccess = true;
    }

    // Calculate Average C/N0
    const auto &sats = m_nmeaParser.getSatellites();
    double totalCn0 = 0.0;
    int count = 0;
    for (auto it = sats.constBegin(); it != sats.constEnd(); ++it) {
        if (it.value().snr > 0) {
            totalCn0 += it.value().snr;
            count++;
        }
    }
    double avgCn0 = count > 0 ? (totalCn0 / count) : 0.0;
    
    qDebug() << QString("Result - PVT Success: %1 | Avg C/N0: %2 dB-Hz")
                .arg(m_currentPvtSuccess ? "YES" : "NO")
                .arg(avgCn0, 0, 'f', 1);

    writeResultToCsv(m_grid[m_currentIndex], m_currentPvtSuccess, avgCn0);

    m_currentIndex++;
    QTimer::singleShot(2000, this, &HeadlessTuner::runNextConfiguration);
}

void HeadlessTuner::onGnssProcessReadyRead()
{
    QByteArray data = m_gnssProcess->readAllStandardOutput() + m_gnssProcess->readAllStandardError();
    QString text = QString::fromUtf8(data);
    QStringList lines = text.split('\n');
    for (const QString &line : lines) {
        if (line.contains("Current receiver time:")) {
            qDebug() << "[GNSS-SDR]" << line.trimmed();
        } else if (line.contains("Position at")) {
            m_currentPvtSuccess = true;
            qDebug() << ">>> PVT LOCK ACHIEVED! <<<";
        }
    }
}

void HeadlessTuner::writeResultToCsv(const TuningConfig &config, bool pvtSuccess, double avgCn0)
{
    QFile csvFile("tuning_results.csv");
    if (csvFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&csvFile);
        out << config.gain << ","
            << config.pllBw << ","
            << config.dllBw << ","
            << (pvtSuccess ? "1" : "0") << ","
            << avgCn0 << "\n";
        csvFile.close();
    }
}

void HeadlessTuner::generateGnssConfig(const TuningConfig &config)
{
    QFile file(m_configPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Failed to open config file for writing";
        return;
    }
    QTextStream out(&file);

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
    out << "SignalSource.gain=" << config.gain << "\n";
    out << "SignalSource.dump=false\n\n";

    out << "SignalConditioner.implementation=Pass_Through\n\n";

    out << "Channels_1C.count=8\n";
    out << "Channels.in_acquisition=4\n\n"; 

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
    out << "Tracking_1C.pll_bw_hz=" << config.pllBw << "\n";
    out << "Tracking_1C.dll_bw_hz=" << config.dllBw << "\n";
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

    file.close();
}
