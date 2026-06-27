#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <QFile>
#include <QMap>
#include <QLabel>
#include <QProgressBar>
#include <QTextEdit>
#include <QComboBox>
#include <QSlider>
#include <QCheckBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include "sdrstreamer.h"
#include "sdrscanner.h"
#include "nmeaparser.h"

struct ChannelState {
    int prn = 0;
    QString status = "Searching";
    qint64 lastUpdate = 0;
    qint64 lockDuration = 0;
};

class SkyplotWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SkyplotWidget(QWidget *parent = nullptr) : QWidget(parent) {}
    
    void setSatellites(const QMap<int, SatelliteInfo> &sats) {
        m_sats = sats;
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QMap<int, SatelliteInfo> m_sats;
};

class SpectrumPlotWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpectrumPlotWidget(QWidget *parent = nullptr) : QWidget(parent) {}
    
    void addPoint(double freq, double power) {
        m_points[freq] = power;
        
        // Update max hold
        if (!m_maxHoldData.contains(freq) || power > m_maxHoldData[freq]) {
            m_maxHoldData[freq] = power;
        }
        
        update();
    }
    
    void clearPoints() {
        m_points.clear();
        m_maxHoldData.clear();
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QMap<double, double> m_points;
    QMap<double, double> m_maxHoldData;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void onStartClicked();
    void onStopClicked();
    void onGainSliderChanged(int value);
    void onPowerMeasured(double powerDb);
    void onSdrLogMessage(const QString &message);
    void onSdrFinished();
    void onGnssProcessReadyRead();
    void onGnssProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onPeriodicUpdate();
    void onDiagnosticsClicked();
    void onAutotuneClicked();
    void onAntennaTestClicked();
    void onAutoTuneTimer();
    
    // Scanner slots
    void onStartScanClicked();
    void onStopScanClicked();
    void onScanPointReceived(double frequency, double powerDb);
    void onInterferenceDetected(double frequency, double powerDb);
    void onScanFinished();

private:
    void setupUi();
    void applyTheme();
    void setupFifo();
    void writeGnssSdrConfig();
    void readGnssLog();
    void readNmeaFile();
    void updateChannelsDisplay();
    void updateSatellitesDisplay();
    void appendLog(const QString &msg, const QString &color = "#a0a0a0");
    void updatePowerLabelDisplay();

    // Backend
    SdrStreamer *m_sdrStreamer;
    SdrScanner *m_sdrScanner;
    QProcess *m_gnssProcess;
    NmeaParser m_nmeaParser;

    // Configuration Paths
    QString m_workspaceDir;
    QString m_fifoPath;
    QString m_configPath;
    QString m_nmeaPath;
    QString m_logPath;

    // Tracking States
    QTimer *m_updateTimer;
    qint64 m_startTime;
    double m_currentPowerDb;
    QString m_receiverTime;
    
    QMap<int, ChannelState> m_channels; // Channel index -> state
    qint64 m_logFilePos;
    qint64 m_nmeaFilePos;

    // UI Widgets
    QPushButton *m_btnStart;
    QPushButton *m_btnStop;
    QPushButton *m_btnDiagnostics;
    QPushButton *m_btnAutotune;
    QPushButton *m_btnAntennaTest;
    QComboBox *m_comboSampleRate;
    QSlider *m_sliderGain;
    QLabel *m_lblGainVal;
    QCheckBox *m_chkBiasTee;
    QCheckBox *m_chkAdaptiveGain;
    QCheckBox *m_chkRecordIq;
    QCheckBox *m_chkAutoTuneTracking;
    
    QDoubleSpinBox *m_spinPllBw;
    QDoubleSpinBox *m_spinDllBw;
    
    int m_lossOfLockCount;
    QTimer *m_autoTuneTimer;
    
    // SNR-based Hill Climbing AGC
    double m_previousAverageCn0;
    int m_agcDirection;
    QTimer *m_snrAgcTimer;
    void onSnrAgcTimer();

    QLabel *m_lblStatus;
    QLabel *m_lblPower;
    QLabel *m_lblTime;

    QLabel *m_lblLatitude;
    QLabel *m_lblLongitude;
    QLabel *m_lblAltitude;
    QLabel *m_lblSpeed;
    QLabel *m_lblFixQuality;
    QLabel *m_lblSatsInFix;
    QLabel *m_lblHdop;
    QLabel *m_lblUtcTime;
    QLabel *m_lblLocalTime;

    QWidget *m_channelsContainer;
    QWidget *m_satsContainer;
    SkyplotWidget *m_skyplot;
    QTextEdit *m_txtConsole;
    
    // Scanner UI Widgets
    QDoubleSpinBox *m_spinScanStart;
    QDoubleSpinBox *m_spinScanStop;
    QDoubleSpinBox *m_spinScanStep;
    QPushButton *m_btnStartScan;
    QPushButton *m_btnStopScan;
    QLabel *m_lblInterferenceWarning;
    SpectrumPlotWidget *m_spectrumPlot;
};

#endif // MAINWINDOW_H
