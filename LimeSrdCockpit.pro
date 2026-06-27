QT       += core gui widgets

TARGET = LimeSrdCockpit
TEMPLATE = app

CONFIG += c++17

# MacPorts / Homebrew search paths for SoapySDR
INCLUDEPATH += /usr/local/include /opt/local/include
LIBS += -L/usr/local/lib -L/opt/local/lib -lSoapySDR

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    sdrstreamer.cpp \
    nmeaparser.cpp

HEADERS += \
    mainwindow.h \
    sdrstreamer.h \
    nmeaparser.h
