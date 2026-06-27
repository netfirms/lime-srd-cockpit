#include "nmeaparser.h"
#include <QStringList>
#include <QDate>
#include <QTime>
#include <QDebug>
#include <cmath>

NmeaParser::NmeaParser()
{
    clear();
}

void NmeaParser::clear()
{
    m_position = GpsPosition();
    m_satellitesSnr.clear();
    m_satellites.clear();
}

double NmeaParser::parseDmToDd(const QString &dm, const QString &hemisphere)
{
    if (dm.isEmpty()) return 0.0;
    
    int dotIndex = dm.indexOf('.');
    if (dotIndex < 2) return 0.0;
    
    QString degStr = dm.left(dotIndex - 2);
    QString minStr = dm.mid(dotIndex - 2);
    
    double deg = degStr.toDouble();
    double min = minStr.toDouble();
    
    double dd = deg + min / 60.0;
    
    if (hemisphere == "S" || hemisphere == "W") {
        dd = -dd;
    }
    
    return dd;
}

QDateTime NmeaParser::parseUtcDateTime(const QString &timeStr, const QString &dateStr)
{
    if (timeStr.length() < 6) return QDateTime();
    
    int hour = timeStr.mid(0, 2).toInt();
    int min = timeStr.mid(2, 2).toInt();
    int sec = timeStr.mid(4, 2).toInt();
    int ms = 0;
    
    if (timeStr.contains('.')) {
        QStringList parts = timeStr.split('.');
        if (parts.size() > 1) {
            ms = parts[1].mid(0, 3).toInt();
        }
    }
    
    QDate qDate;
    if (dateStr.length() == 6) {
        int day = dateStr.mid(0, 2).toInt();
        int month = dateStr.mid(2, 2).toInt();
        int year = dateStr.mid(4, 2).toInt() + 2000;
        qDate = QDate(year, month, day);
    } else {
        qDate = QDate::currentDate(); // Fallback
    }
    
    QTime qTime(hour, min, sec, ms);
    return QDateTime(qDate, qTime, Qt::UTC);
}

bool NmeaParser::parseLine(const QString &line)
{
    QString trimmed = line.trimmed();
    if (!trimmed.startsWith("$")) return false;
    
    int starIdx = trimmed.indexOf('*');
    QString payload = trimmed;
    if (starIdx != -1) {
        payload = trimmed.left(starIdx);
    }
    
    QStringList parts = payload.split(',');
    if (parts.isEmpty()) return false;
    
    QString type = parts[0];
    
    if (type.endsWith("GGA")) {
        if (parts.size() >= 10) {
            m_position.rawTime = parts[1];
            
            int quality = parts[6].toInt();
            m_position.fixQuality = quality;
            m_position.hasFix = (quality > 0);
            m_position.numSatsInFix = parts[7].toInt();
            m_position.hdop = parts[8].isEmpty() ? 99.9 : parts[8].toDouble();
            
            if (m_position.hasFix) {
                m_position.latitude = parseDmToDd(parts[2], parts[3]);
                m_position.longitude = parseDmToDd(parts[4], parts[5]);
                
                double rawAlt = parts[9].toDouble();
                double geoSep = 0.0;
                if (parts.size() >= 12 && !parts[11].isEmpty()) {
                    geoSep = parts[11].toDouble();
                }
                
                if (std::abs(geoSep) < 0.001) {
                    double lat = m_position.latitude;
                    double lon = m_position.longitude;
                    if (lat >= 5.0 && lat <= 21.0 && lon >= 97.0 && lon <= 106.0) {
                        double estGeoid = -43.8 - (lat - 7.88) * 0.31 + (lon - 98.40) * 0.15;
                        m_position.altitude = rawAlt - estGeoid;
                    } else {
                        m_position.altitude = rawAlt;
                    }
                } else {
                    m_position.altitude = rawAlt;
                }
            }
            
            if (!m_position.rawTime.isEmpty()) {
                m_position.utcDateTime = parseUtcDateTime(m_position.rawTime, m_position.rawDate);
            }
            return true;
        }
    } 
    else if (type.endsWith("RMC")) {
        if (parts.size() >= 10) {
            m_position.rawTime = parts[1];
            m_position.rawDate = parts[9];
            
            bool active = (parts[2] == "A");
            if (active && parts.size() >= 8) {
                m_position.latitude = parseDmToDd(parts[3], parts[4]);
                m_position.longitude = parseDmToDd(parts[5], parts[6]);
                m_position.speedKnots = parts[7].toDouble();
            }
            
            if (!m_position.rawTime.isEmpty() && !m_position.rawDate.isEmpty()) {
                m_position.utcDateTime = parseUtcDateTime(m_position.rawTime, m_position.rawDate);
            }
            return true;
        }
    }
    else if (type.endsWith("GSV")) {
        if (parts.size() >= 8) {
            for (int i = 4; i < parts.size() - 3; i += 4) {
                if (i + 3 < parts.size()) {
                    QString prnStr = parts[i];
                    QString eleStr = parts[i + 1];
                    QString aziStr = parts[i + 2];
                    QString snrStr = parts[i + 3];
                    if (!prnStr.isEmpty()) {
                        int prn = prnStr.toInt();
                        if (prn > 0) {
                            int snrVal = snrStr.isEmpty() ? 0 : snrStr.toInt();
                            m_satellitesSnr[prn] = snrVal;
                            
                            SatelliteInfo sat;
                            sat.prn = prn;
                            sat.elevation = eleStr.isEmpty() ? 0 : eleStr.toInt();
                            sat.azimuth = aziStr.isEmpty() ? 0 : aziStr.toInt();
                            sat.snr = snrVal;
                            m_satellites[prn] = sat;
                        }
                    }
                }
            }
            return true;
        }
    }
    
    return false;
}
