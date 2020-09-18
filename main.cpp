#include "mainwindow.h"
#include <QApplication>

#include <iostream>
#include <string>

void test_splines();


using namespace std;

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;

    // I have different applications
    // they can be switched in te constructor of the ogl widget

    w.show();

    return a.exec();
}
