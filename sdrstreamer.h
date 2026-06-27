#ifndef SDRSTREAMER_H
#define SDRSTREAMER_H

#include <QThread>
#include <QString>
#include <atomic>

class SdrStreamer : public QThread
{
    Q_OBJECT
public:
    explicit SdrStreamer(QObject *parent = nullptr);
    ~SdrStreamer();

    void startStreaming(const QString &fifoPath, double sampleRate, double gain, bool enableBiasTee, bool recordIq);
    void stopStreaming();
    void setDynamicGain(double gain);
    bool isRunningStream() const { return m_running; }

signals:
    void powerMeasured(double powerDb);
    void logMessage(const QString &message);
    void finishedStreaming();

protected:
    void run() override;

private:
    QString m_fifoPath;
    double m_sampleRate;
    std::atomic<double> m_gain;
    bool m_enableBiasTee;
    bool m_recordIq;
    std::atomic<bool> m_running;
};

#endif // SDRSTREAMER_H
