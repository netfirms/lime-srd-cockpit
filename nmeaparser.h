#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <QString>
#include <QMap>
#include <QDateTime>

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
    
    void clear();

private:
    GpsPosition m_position;
    QMap<int, int> m_satellitesSnr; // Maps PRN -> SNR (dB-Hz)

    double parseDmToDd(const QString &dm, const QString &hemisphere);
    QDateTime parseUtcDateTime(const QString &timeStr, const QString &dateStr);
};

#endif // NMEAPARSER_H
