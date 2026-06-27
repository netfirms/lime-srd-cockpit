#include "antennatestdialog.h"
#include <QMessageBox>
#include <QDir>
#include <QFile>

AntennaTestDialog::AntennaTestDialog(QWidget *parent) : QDialog(parent), m_state(INIT), m_powerPlugged(0.0), m_powerUnplugged(0.0)
{
    setWindowTitle("Antenna Noise Floor Test");
    setMinimumWidth(400);

    QVBoxLayout *layout = new QVBoxLayout(this);

    m_lblInstructions = new QLabel("Welcome to the Antenna LNA test.\n\nThis will verify if your active GPS antenna is receiving power and generating RF noise.", this);
    m_lblInstructions->setWordWrap(true);
    
    m_lblStatus = new QLabel("", this);
    m_lblStatus->setAlignment(Qt::AlignCenter);
    m_lblStatus->setStyleSheet("font-weight: bold; color: #3b82f6;"); // Blue

    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 3);
    m_progressBar->setValue(0);
    m_progressBar->setTextVisible(false);

    m_btnAction = new QPushButton("Start Test", this);
    m_btnCancel = new QPushButton("Cancel", this);

    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(m_btnCancel);
    btnLayout->addWidget(m_btnAction);

    layout->addWidget(m_lblInstructions);
    layout->addSpacing(10);
    layout->addWidget(m_lblStatus);
    layout->addWidget(m_progressBar);
    layout->addSpacing(10);
    layout->addLayout(btnLayout);

    connect(m_btnAction, &QPushButton::clicked, this, &AntennaTestDialog::onActionClicked);
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    m_streamer = new SdrStreamer();
    connect(m_streamer, &SdrStreamer::powerMeasured, this, &AntennaTestDialog::onPowerMeasured);

    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setInterval(1000);
    connect(m_timeoutTimer, &QTimer::timeout, this, &AntennaTestDialog::onMeasurementTimeout);

    nextState();
}

AntennaTestDialog::~AntennaTestDialog()
{
    if (m_streamer->isRunningStream()) {
        m_streamer->stopStreaming();
        m_streamer->wait();
    }
    delete m_streamer;
}

void AntennaTestDialog::nextState()
{
    switch (m_state) {
        case INIT:
            m_lblInstructions->setText("<b>STEP 1:</b><br>Please ensure your GPS antenna is securely <b>PLUGGED IN</b> to the LimeSDR-USB (RX1_H port).");
            m_btnAction->setText("Ready (Start Measurement)");
            m_progressBar->setValue(0);
            m_lblStatus->setText("Waiting for user...");
            m_state = MEASURING_PLUGGED;
            break;
            
        case MEASURING_PLUGGED:
            startMeasurement();
            m_lblInstructions->setText("Measuring baseline RF noise floor...\nPlease do not touch the antenna.");
            break;
            
        case WAITING_UNPLUG:
            m_lblInstructions->setText("<b>STEP 2:</b><br>Please completely <b>UNPLUG</b> the GPS antenna from the LimeSDR-USB.");
            m_btnAction->setText("Ready (Start Measurement)");
            m_btnAction->setEnabled(true);
            m_btnCancel->setEnabled(true);
            m_progressBar->setValue(0);
            m_lblStatus->setText(QString("Plugged-in noise floor: %1 dBFS").arg(m_powerPlugged, 0, 'f', 2));
            m_state = MEASURING_UNPLUGGED;
            break;
            
        case MEASURING_UNPLUGGED:
            startMeasurement();
            m_lblInstructions->setText("Measuring unplugged RF noise floor...\nPlease wait.");
            break;
            
        case FINISHED:
            m_btnAction->setText("Finish");
            m_btnAction->setEnabled(true);
            m_btnCancel->setEnabled(false);
            m_progressBar->setValue(3);
            
            double diff = m_powerPlugged - m_powerUnplugged;
            QString resultText = QString("Plugged In: %1 dBFS\nUnplugged: %2 dBFS\n\nDifference: %3 dB\n\n").arg(m_powerPlugged, 0, 'f', 2).arg(m_powerUnplugged, 0, 'f', 2).arg(diff, 0, 'f', 2);
            
            if (diff > 1.0) {
                resultText += "<b>TEST PASSED!</b><br>Your antenna LNA is successfully receiving power and generating a noise floor drop when unplugged.";
                m_lblStatus->setStyleSheet("font-weight: bold; color: #10b981;"); // Green
            } else {
                resultText += "<b>TEST FAILED!</b><br>No significant noise floor drop detected. The antenna is not drawing power or is broken. Check the Bias-Tee.";
                m_lblStatus->setStyleSheet("font-weight: bold; color: #ef4444;"); // Red
            }
            
            m_lblInstructions->setText(resultText);
            m_lblStatus->setText(diff > 1.0 ? "LNA Working!" : "LNA Not Functioning!");
            break;
    }
}

void AntennaTestDialog::onActionClicked()
{
    if (m_state == FINISHED) {
        accept();
        return;
    }
    
    nextState();
}

void AntennaTestDialog::startMeasurement()
{
    m_btnAction->setEnabled(false);
    m_btnCancel->setEnabled(false);
    m_measurements.clear();
    m_progressBar->setValue(0);
    
    QString dummyFile = "/dev/null";
    
    if (!m_streamer->isRunningStream()) {
        m_streamer->startStreaming(dummyFile, 2000000, 60.0, true, false);
    }
    
    m_lblStatus->setText("Measuring...");
    m_timeoutTimer->start();
}

void AntennaTestDialog::onPowerMeasured(double powerDb)
{
    if (m_state == MEASURING_PLUGGED || m_state == MEASURING_UNPLUGGED) {
        m_measurements.append(powerDb);
    }
}

void AntennaTestDialog::onMeasurementTimeout()
{
    int count = m_measurements.size();
    m_progressBar->setValue(count);
    
    if (count >= 3) {
        m_timeoutTimer->stop();
        
        // Stop streaming
        if (m_streamer->isRunningStream()) {
            m_streamer->stopStreaming();
            m_streamer->wait();
        }
        
        // Average measurements
        double sum = 0;
        for (double p : m_measurements) sum += p;
        double avg = sum / count;
        
        if (m_state == MEASURING_PLUGGED) {
            m_powerPlugged = avg;
            m_state = WAITING_UNPLUG;
            nextState();
        } else if (m_state == MEASURING_UNPLUGGED) {
            m_powerUnplugged = avg;
            m_state = FINISHED;
            nextState();
        }
    }
}
