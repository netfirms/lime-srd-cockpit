#ifndef HEADLESSTUNER_H
#define HEADLESSTUNER_H

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVector>
#include "sdrstreamer.h"
#include "nmeaparser.h"

struct TuningConfig {
    int gain;
    double pllBw;
    double dllBw;
};

class HeadlessTuner : public QObject
{
    Q_OBJECT
public:
    explicit HeadlessTuner(QObject *parent = nullptr);
    void startTuning();

private slots:
    void runNextConfiguration();
    void onTestTimeout();
    void onGnssProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onGnssProcessReadyRead();

private:
    void generateGnssConfig(const TuningConfig &config);
    void writeResultToCsv(const TuningConfig &config, bool pvtSuccess, double avgCn0);

    QVector<TuningConfig> m_grid;
    int m_currentIndex;

    QProcess *m_gnssProcess;
    SdrStreamer *m_sdrStreamer;
    NmeaParser m_nmeaParser;

    QTimer *m_testTimer;
    
    QString m_fifoPath;
    QString m_configPath;
    
    bool m_currentPvtSuccess;
};

#endif // HEADLESSTUNER_H
