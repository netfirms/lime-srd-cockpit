#include "sdrstreamer.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <QFile>
#include <QFileInfo>
#include <QDebug>
#include <cmath>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstring>
#include <cerrno>

SdrStreamer::SdrStreamer(QObject *parent)
    : QThread(parent)
    , m_sampleRate(2e6)
    , m_gain(60.0)
    , m_enableBiasTee(false)
    , m_recordIq(false)
    , m_running(false)
{
}

SdrStreamer::~SdrStreamer()
{
    stopStreaming();
    wait();
}

void SdrStreamer::startStreaming(const QString &fifoPath, double sampleRate, double gain, bool enableBiasTee, bool recordIq)
{
    m_fifoPath = fifoPath;
    m_sampleRate = sampleRate;
    m_gain = gain;
    m_enableBiasTee = enableBiasTee;
    m_recordIq = recordIq;
    m_running = true;
    start();
}

void SdrStreamer::stopStreaming()
{
    m_running = false;
}

void SdrStreamer::setDynamicGain(double gain)
{
    m_gain = gain;
}

void SdrStreamer::run()
{
    emit logMessage("Starting SDR Thread...");
    
    // Set soapy plugin path for Mac OS
    setenv("SOAPY_SDR_PLUGIN_PATH", "/usr/local/lib/SoapySDR/modules0.8", 0);
    
    SoapySDR::Device *sdr = nullptr;
    try {
        emit logMessage("Connecting to LimeSDR device...");
        sdr = SoapySDR::Device::make(); // finds first device
        if (!sdr) {
            emit logMessage("ERROR: No SDR device found!");
            emit finishedStreaming();
            return;
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR connecting to device: %1").arg(e.what()));
        emit finishedStreaming();
        return;
    }

    emit logMessage(QString("Connected to %1 (%2)").arg(
        QString::fromStdString(sdr->getDriverKey()), 
        QString::fromStdString(sdr->getHardwareKey())
    ));

    // Configure SDR parameters
    try {
        sdr->setSampleRate(SOAPY_SDR_RX, 0, m_sampleRate);
        sdr->setBandwidth(SOAPY_SDR_RX, 0, 5.0e6); // 5.0 MHz analog bandwidth
        
        // Offset tuning to eliminate DC spike in the GPS band
        sdr->setFrequency(SOAPY_SDR_RX, 0, "RF", 1573.42e6);
        sdr->setFrequency(SOAPY_SDR_RX, 0, "BB", 2.0e6);
        
        sdr->setAntenna(SOAPY_SDR_RX, 0, "LNAH"); // RX0 port (RX1_H on case)
        sdr->setGain(SOAPY_SDR_RX, 0, m_gain);
        
        // Enable Bias Tee if requested
        if (m_enableBiasTee) {
            emit logMessage("Attempting to enable Bias-Tee (active antenna power)...");
            try {
                sdr->writeSetting("bias_tee", "true");
                emit logMessage("Bias-Tee enabled successfully.");
            } catch (...) {
                try {
                    sdr->writeSetting("BIAS_TEE", "true");
                    emit logMessage("Bias-Tee enabled successfully (BIAS_TEE key).");
                } catch (const std::exception &e) {
                    emit logMessage(QString("WARNING: Could not enable Bias-Tee: %1. External power might be required.").arg(e.what()));
                }
            }
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR setting up SDR parameters: %1").arg(e.what()));
        SoapySDR::Device::unmake(sdr);
        emit finishedStreaming();
        return;
    }

    // Setup Stream
    SoapySDR::Stream *rxStream = nullptr;
    try {
        rxStream = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
        if (!rxStream) {
            emit logMessage("ERROR: Failed to setup RX stream!");
            SoapySDR::Device::unmake(sdr);
            emit finishedStreaming();
            return;
        }
        sdr->activateStream(rxStream);
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR setting up/activating stream: %1").arg(e.what()));
        if (rxStream) sdr->closeStream(rxStream);
        SoapySDR::Device::unmake(sdr);
        emit finishedStreaming();
        return;
    }

    emit logMessage(QString("Streaming started. Writing to FIFO: %1").arg(m_fifoPath));

    // Open FIFO for writing
    // We open in O_RDWR (read-write) to prevent blocking if there is no reader yet.
    int fd = ::open(m_fifoPath.toUtf8().constData(), O_RDWR);
    if (fd < 0) {
        emit logMessage(QString("ERROR: Failed to open FIFO %1: %2").arg(m_fifoPath, strerror(errno)));
        sdr->deactivateStream(rxStream);
        sdr->closeStream(rxStream);
        SoapySDR::Device::unmake(sdr);
        emit finishedStreaming();
        return;
    }

    QFile recordFile;
    if (m_recordIq) {
        recordFile.setFileName("limesdr_raw_gps.bin");
        if (recordFile.open(QIODevice::WriteOnly)) {
            emit logMessage("Started recording raw IQ samples to limesdr_raw_gps.bin");
        } else {
            emit logMessage("ERROR: Failed to open limesdr_raw_gps.bin for recording!");
        }
    }

    const size_t bufferSize = 8192;
    std::vector<std::complex<float>> buffer(bufferSize);
    void *buffs[] = { buffer.data() };
    
    int lastPowerPrintTime = 0;
    double lastAppliedGain = m_gain;
    
    // Core Streaming Loop
    while (m_running) {
        // Check for dynamic gain changes
        double currentGain = m_gain;
        if (std::abs(currentGain - lastAppliedGain) > 0.1) {
            try {
                sdr->setGain(SOAPY_SDR_RX, 0, currentGain);
                lastAppliedGain = currentGain;
                emit logMessage(QString("[SDR] Dynamic gain updated to %1 dB").arg(currentGain));
            } catch (const std::exception &e) {
                emit logMessage(QString("[SDR] WARNING: Failed to update dynamic gain: %1").arg(e.what()));
            }
        }

        int flags = 0;
        long long timeNs = 0;
        int ret = sdr->readStream(rxStream, buffs, bufferSize, flags, timeNs, 100000); // 100ms timeout
        
        if (ret > 0) {
            // Write to FIFO
            size_t bytesToWrite = ret * sizeof(std::complex<float>);
            ssize_t written = ::write(fd, buffer.data(), bytesToWrite);
            if (written < 0) {
                if (errno == EPIPE) {
                    emit logMessage("FIFO Pipe closed by reader. Stopping SDR Stream.");
                    break;
                }
            }

            // Write raw IQ to record file if enabled
            if (recordFile.isOpen()) {
                recordFile.write(reinterpret_cast<const char*>(buffer.data()), bytesToWrite);
            }

            // Compute power periodically
            lastPowerPrintTime += ret;
            if (lastPowerPrintTime >= m_sampleRate) { // Approx once per second
                double sumMagSq = 0;
                for (int i = 0; i < ret; ++i) {
                    float mag = std::abs(buffer[i]);
                    sumMagSq += mag * mag;
                }
                double meanMagSq = sumMagSq / ret;
                double powerDb = 10.0 * std::log10(meanMagSq + 1e-12);
                emit powerMeasured(powerDb);
                lastPowerPrintTime = 0;
            }
        } else if (ret == SOAPY_SDR_TIMEOUT) {
            continue;
        } else if (ret == SOAPY_SDR_OVERFLOW) {
            emit logMessage("WARNING: SDR Stream Overflow!");
        } else {
            emit logMessage(QString("ERROR reading from SDR stream: %1").arg(ret));
            break;
        }
    }

    // Cleanup
    emit logMessage("Stopping SDR stream...");
    if (recordFile.isOpen()) {
        recordFile.close();
        emit logMessage("Stopped recording raw IQ samples.");
    }
    ::close(fd);
    sdr->deactivateStream(rxStream);
    sdr->closeStream(rxStream);
    SoapySDR::Device::unmake(sdr);
    emit logMessage("SDR Thread exit.");
    emit finishedStreaming();
}
