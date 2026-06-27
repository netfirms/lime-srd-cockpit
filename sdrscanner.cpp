#include "sdrscanner.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <QDebug>
#include <cmath>
#include <unistd.h>
#include <cstdlib>

SdrScanner::SdrScanner(QObject *parent)
    : QThread(parent)
    , m_startFreq(1500e6)
    , m_stopFreq(1600e6)
    , m_stepFreq(1e6)
    , m_gain(60.0)
    , m_enableBiasTee(false)
    , m_scanning(false)
{
}

SdrScanner::~SdrScanner()
{
    stopScanning();
    wait();
}

void SdrScanner::startScanning(double startFreq, double stopFreq, double stepFreq, double gain, bool enableBiasTee)
{
    m_startFreq = startFreq;
    m_stopFreq = stopFreq;
    m_stepFreq = stepFreq;
    m_gain = gain;
    m_enableBiasTee = enableBiasTee;
    m_scanning = true;
    start();
}

void SdrScanner::stopScanning()
{
    m_scanning = false;
}

void SdrScanner::run()
{
    emit logMessage("Starting SDR Scanner Thread...");
    
    setenv("SOAPY_SDR_PLUGIN_PATH", "/usr/local/lib/SoapySDR/modules0.8", 0);
    
    SoapySDR::Device *sdr = nullptr;
    try {
        emit logMessage("Connecting to LimeSDR device for scanning...");
        sdr = SoapySDR::Device::make();
        if (!sdr) {
            emit logMessage("ERROR: No SDR device found!");
            emit scanFinished();
            return;
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR connecting to device: %1").arg(e.what()));
        emit scanFinished();
        return;
    }

    emit logMessage(QString("Scanner connected to %1 (%2)").arg(
        QString::fromStdString(sdr->getDriverKey()), 
        QString::fromStdString(sdr->getHardwareKey())
    ));

    try {
        // We can use a lower sample rate for scanning to speed up tuning/reading
        double sampleRate = 2e6;
        sdr->setSampleRate(SOAPY_SDR_RX, 0, sampleRate);
        sdr->setBandwidth(SOAPY_SDR_RX, 0, sampleRate);
        
        sdr->setAntenna(SOAPY_SDR_RX, 0, "LNAH");
        
        double totalGain = m_gain;
        try {
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
        } catch (...) {
            sdr->setGain(SOAPY_SDR_RX, 0, totalGain);
        }
        
        if (m_enableBiasTee) {
            try {
                sdr->writeSetting("bias_tee", "true");
            } catch (...) {
                try {
                    sdr->writeSetting("BIAS_TEE", "true");
                } catch (...) {}
            }
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR setting up SDR parameters: %1").arg(e.what()));
        SoapySDR::Device::unmake(sdr);
        emit scanFinished();
        return;
    }

    SoapySDR::Stream *rxStream = nullptr;
    try {
        rxStream = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
        if (!rxStream) {
            emit logMessage("ERROR: Failed to setup RX stream!");
            SoapySDR::Device::unmake(sdr);
            emit scanFinished();
            return;
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("ERROR setting up stream: %1").arg(e.what()));
        if (rxStream) sdr->closeStream(rxStream);
        SoapySDR::Device::unmake(sdr);
        emit scanFinished();
        return;
    }

    const size_t bufferSize = 8192;
    std::vector<std::complex<float>> buffer(bufferSize);
    void *buffs[] = { buffer.data() };
    
    // Activate stream ONCE to prevent libusb abort_transfers spam on macOS
    sdr->activateStream(rxStream);
    
    // Perform continuous scan
    while (m_scanning) {
        double currentFreq = m_startFreq;
        QList<double> sweepPowers;
        QList<double> sweepFreqs;
        
        while (m_scanning && currentFreq <= m_stopFreq) {
            try {
                sdr->setFrequency(SOAPY_SDR_RX, 0, "RF", currentFreq);
                
                // Allow PLL to settle and calibration to run
                usleep(30000); 
                
                int flags = 0;
                long long timeNs = 0;
                
                // Flush stale samples from hardware FIFO that were captured during tuning/calibration
                for (int k = 0; k < 3; ++k) {
                    sdr->readStream(rxStream, buffs, bufferSize, flags, timeNs, 100000);
                }
                
                // Read fresh samples for measurement
                int ret = sdr->readStream(rxStream, buffs, bufferSize, flags, timeNs, 100000);
                
                if (ret > 0) {
                    // Remove DC offset (DC leak) from the calculation
                    std::complex<float> sum(0, 0);
                    for (int i = 0; i < ret; ++i) {
                        sum += buffer[i];
                    }
                    std::complex<float> mean = sum / (float)ret;
                    
                    double sumMagSq = 0;
                    for (int i = 0; i < ret; ++i) {
                        std::complex<float> ac = buffer[i] - mean; // AC coupling
                        float mag = std::abs(ac);
                        sumMagSq += mag * mag;
                    }
                    double meanMagSq = sumMagSq / ret;
                    double powerDb = 10.0 * std::log10(meanMagSq + 1e-12);
                    emit scanPoint(currentFreq, powerDb);
                    
                    sweepPowers.append(powerDb);
                    sweepFreqs.append(currentFreq);
                } else {
                     emit logMessage(QString("WARNING: Stream read timeout at %1 MHz").arg(currentFreq / 1e6));
                }
            } catch (const std::exception &e) {
                emit logMessage(QString("ERROR during scan at %1 Hz: %2").arg(currentFreq).arg(e.what()));
                m_scanning = false; // Break outer loop on critical error
                break;
            }
            
            currentFreq += m_stepFreq;
        }
        
        // Interference Detection Analysis
        if (m_scanning && sweepPowers.size() > 5) {
            QList<double> sorted = sweepPowers;
            std::sort(sorted.begin(), sorted.end());
            double medianPower = sorted[sorted.size() / 2];
            
            for (int i = 0; i < sweepPowers.size(); ++i) {
                // Ignore the center frequency DC spike if it happens to still leak
                if (std::abs(sweepFreqs[i] - 1575.42e6) < 1.0) continue; 
                
                if (sweepPowers[i] > medianPower + 15.0) { // 15 dB threshold
                    emit interferenceDetected(sweepFreqs[i], sweepPowers[i]);
                    break; // Just report the first/strongest one per sweep to avoid spam
                }
            }
        }
        
        // Brief pause between full sweeps to allow UI to breathe
        if (m_scanning) {
            usleep(50000); // 50 ms
        }
    }
    
    sdr->deactivateStream(rxStream);

    emit logMessage("Stopping SDR Scanner stream...");
    sdr->closeStream(rxStream);
    SoapySDR::Device::unmake(sdr);
    emit logMessage("SDR Scanner Thread exit.");
    emit scanFinished();
}
