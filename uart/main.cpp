#include "uart.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    uart w;
    w.show();

    return a.exec();
}
