#ifndef UART_H
#define UART_H

#include <QWidget>

namespace Ui {
class uart;
}

class uart : public QWidget
{
    Q_OBJECT

public:
    explicit uart(QWidget *parent = 0);
    ~uart();

private slots:
    void on_uart_send_clicked();

    void on_uatr_on_clicked();

    void on_uart_off_clicked();

    void on_uart_recv_clicked();

private:
    Ui::uart *ui;
};

#endif // UART_H
