#ifndef ANTENNATESTDIALOG_H
#define ANTENNATESTDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QTimer>
#include <QVector>
#include "sdrstreamer.h"

class AntennaTestDialog : public QDialog
{
    Q_OBJECT
public:
    explicit AntennaTestDialog(QWidget *parent = nullptr);
    ~AntennaTestDialog();

private slots:
    void onActionClicked();
    void onPowerMeasured(double powerDb);
    void onMeasurementTimeout();

private:
    void nextState();
    void startMeasurement();

    enum TestState {
        INIT,
        MEASURING_PLUGGED,
        WAITING_UNPLUG,
        MEASURING_UNPLUGGED,
        FINISHED
    };

    TestState m_state;
    SdrStreamer *m_streamer;
    
    QLabel *m_lblInstructions;
    QLabel *m_lblStatus;
    QProgressBar *m_progressBar;
    QPushButton *m_btnAction;
    QPushButton *m_btnCancel;

    QTimer *m_timeoutTimer;
    
    QVector<double> m_measurements;
    double m_powerPlugged;
    double m_powerUnplugged;
};

#endif // ANTENNATESTDIALOG_H
