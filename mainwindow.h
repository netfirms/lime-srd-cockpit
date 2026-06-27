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
#include "sdrstreamer.h"
#include "nmeaparser.h"

struct ChannelState {
    int prn = 0;
    QString status = "Searching";
    qint64 lastUpdate = 0;
    qint64 lockDuration = 0;
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

    // Backend
    SdrStreamer *m_sdrStreamer;
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
    QComboBox *m_comboSampleRate;
    QSlider *m_sliderGain;
    QLabel *m_lblGainVal;
    QCheckBox *m_chkBiasTee;

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
    QTextEdit *m_txtConsole;
};

#endif // MAINWINDOW_H
