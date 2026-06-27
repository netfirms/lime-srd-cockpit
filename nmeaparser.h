#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <QString>
#include <QMap>
#include <QDateTime>

struct SatelliteInfo {
    int prn = 0;
    int elevation = 0;   // 0 to 90 degrees
    int azimuth = 0;     // 0 to 359 degrees
    int snr = 0;         // C/N0 dB-Hz
};

struct GpsPosition {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    bool hasFix = false;
    int fixQuality = 0;
    int numSatsInFix = 0;
    double hdop = 99.9;
    double speedKnots = 0.0;
    QDateTime utcDateTime;
    QString rawTime;
    QString rawDate;
};

class NmeaParser
{
public:
    NmeaParser();
    
    bool parseLine(const QString &line);
    
    GpsPosition getPosition() const { return m_position; }
    QMap<int, int> getSatelliteSnr() const { return m_satellitesSnr; }
    QMap<int, SatelliteInfo> getSatellites() const { return m_satellites; }
    
    void clear();

private:
    GpsPosition m_position;
    QMap<int, int> m_satellitesSnr; // Maps PRN -> SNR (dB-Hz)
    QMap<int, SatelliteInfo> m_satellites; // Maps PRN -> SatelliteInfo

    double parseDmToDd(const QString &dm, const QString &hemisphere);
    QDateTime parseUtcDateTime(const QString &timeStr, const QString &dateStr);
};

#endif // NMEAPARSER_H
