#include <QApplication>
#include <QTimer>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    bool autostart = false;
    for (int i = 1; i < argc; ++i) {
        if (QString(argv[i]) == "--autostart") {
            autostart = true;
        }
    }
    
    MainWindow w;
    w.show();
    
    if (autostart) {
        QTimer::singleShot(500, &w, &MainWindow::onStartClicked);
    }
    
    return a.exec();
}
