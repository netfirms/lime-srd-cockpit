#include <QApplication>
#include <QTimer>
#include <signal.h>
#include "mainwindow.h"
#include "headlesstuner.h"

int main(int argc, char *argv[])
{
    signal(SIGPIPE, SIG_IGN);
    bool headless = false;
    bool autostart = false;
    for (int i = 1; i < argc; ++i) {
        if (QString(argv[i]) == "--autostart") {
            autostart = true;
        } else if (QString(argv[i]) == "--headless-tune") {
            headless = true;
        }
    }
    
    if (headless) {
        QCoreApplication a(argc, argv);
        HeadlessTuner tuner;
        QTimer::singleShot(0, &tuner, &HeadlessTuner::startTuning);
        return a.exec();
    } else {
        QApplication a(argc, argv);
        MainWindow w;
        w.show();
        
        if (autostart) {
            QTimer::singleShot(500, &w, &MainWindow::onStartClicked);
        }
        
        return a.exec();
    }
}
