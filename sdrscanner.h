#ifndef SDRSCANNER_H
#define SDRSCANNER_H

#include <QThread>
#include <QString>
#include <atomic>

class SdrScanner : public QThread
{
    Q_OBJECT
public:
    explicit SdrScanner(QObject *parent = nullptr);
    ~SdrScanner();

    void startScanning(double startFreq, double stopFreq, double stepFreq, double gain, bool enableBiasTee);
    void stopScanning();
    bool isScanning() const { return m_scanning; }

signals:
    void scanPoint(double frequency, double powerDb);
    void interferenceDetected(double frequency, double powerDb);
    void scanFinished();
    void logMessage(const QString &message);

protected:
    void run() override;

private:
    double m_startFreq;
    double m_stopFreq;
    double m_stepFreq;
    double m_gain;
    bool m_enableBiasTee;
    std::atomic<bool> m_scanning;
};

#endif // SDRSCANNER_H
