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
    sdrscanner.cpp \
    nmeaparser.cpp \
    headlesstuner.cpp \
    antennatestdialog.cpp

HEADERS += \
    mainwindow.h \
    sdrstreamer.h \
    sdrscanner.h \
    nmeaparser.h \
    headlesstuner.h \
    antennatestdialog.h

run.target = run
run.commands = ./LimeSrdCockpit.app/Contents/MacOS/LimeSrdCockpit
run.depends = $(TARGET)
QMAKE_EXTRA_TARGETS += run
