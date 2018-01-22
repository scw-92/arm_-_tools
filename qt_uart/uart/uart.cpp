#include "uart.h"
#include "ui_uart.h"
#include <QString>
#include <QDialog>

//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
//#include <libwebsockets.h>
//#include <jsoncpp/json/json.h>
//#include <json/json.h>
#include<fcntl.h>
#include<assert.h>
#include<termios.h>
#include<sys/types.h>
#include<errno.h>

#define UART_ON 1
#define UART_OFF 2
#define UART_SEND 3
#define UART_READ 4


int uart_send_en = 0;//uart能否发送标志位
//int web_recv_flags=-1;//ws recv con_bit;

int ret;
int uart_fd= -1;                //uart_file文件描述符
char uart_buf_read[1024];      //读取反冲去
char  uart_buf_write[1024];    //写入缓冲区
int uart_read_en = 0;          //读使能标志位
int uart_on =0;                //uart打开标志位
int uart_send_auto = 0;       //是否自动发送标志位
int uart_send_time = 0;      //定时发送时间
//QDialog dialog_pthread;      //线程专属对话框
//QDialog dialog_pthread_read;      //线程专属对话框


int pthread_on = 0;//线程是否开启线程标志位
QTextEdit *read_text;


int uart_write(int fd,const char *w_buf,size_t len,Ui::uart *ui);
int uart_set(int uart_fd,int baude,int c_flow,int bits,int parity,int stop,Ui::uart *ui);
int uart_read(int fd,char *r_buf,size_t len,Ui::uart *ui);
int uart_open(int uart_fd,const char *pathname,Ui::uart *ui);
ssize_t safe_write(int uart_fd, const void *vptr, size_t n,Ui::uart *ui);
int uart_main(const char* file_uart,int baude,int c_flow,int bits,int  parity,int stop,Ui::uart *ui);
int uart_close(int uart_fd,Ui::uart *ui);
ssize_t safe_read(int uart_fd,char *vptr,size_t n,Ui::uart *ui);
void* pthread_read(void * arg);
void * pthread_write(void *arg);

void* pthread_read(void * arg)
{
    Ui::uart *ui = (Ui::uart *)arg;
    while(1)
    {
        if(uart_on)
        {
            if(uart_read_en)
            {

                if(uart_on && uart_read_en)
                {
                    //printf("---->%d\n",__LINE__);
                    ret = uart_read(uart_fd,uart_buf_read,1024,ui);
                    if(ret == -1)
                     {
                            ui->uart_err_box->setText("uart_read error");
                     }

                }
            }
        }
        sleep(1);
    }
    return (void *)0;
}

void * pthread_write(void *arg)
{
    Ui::uart *ui = (Ui::uart *)arg;
    while(1)
    {
        if(uart_on)
        {
            if(uart_send_en)
            {
                if(strlen(uart_buf_write))
                {
                    if(uart_send_auto)
                    {
                        printf("-->%d\n",98);

                        ret = uart_write(uart_fd,uart_buf_write,strlen(uart_buf_write),ui);
                        if(ret == -1)
                          {

                          ui->uart_err_box->setText("uart_write error");

                        }
                        usleep(uart_send_time);
                    }else
                    {
                        ret = uart_write(uart_fd,uart_buf_write,strlen(uart_buf_write),ui);
                        printf("-->%d\n",111);
                        if(ret == -1)
                          {
                           ui->uart_err_box->setText("uart_write error");;
                        }
                       // sleep(1);
                    }
                }
            }
        }
   }
    return (void *)0;
}



ssize_t safe_write(int uart_fd, const void *vptr, size_t n,Ui::uart *ui)
{
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = (char *)vptr;
    nleft = n;
    printf("safe_write+135\n");

    while(nleft > 0)
    {
    if((nwritten = write(uart_fd, ptr, nleft)) <= 0)
        {
            if(nwritten < 0&&errno == EINTR)//被中断打断
                nwritten = 0;
            else
                return -1;
        }
        nleft -= nwritten;
        ptr   += nwritten;
        //printf("-->%ld\n",nwritten);
    }
    printf("safe_write%s-->%d\n",vptr,__LINE__);
    if(uart_send_auto==0)
    {
        bzero((void *)vptr,1024);
        uart_send_en = 0;
    }
    return(n);
}

ssize_t safe_read(int uart_fd,char *vptr,size_t n,Ui::uart *ui)
{
    size_t nleft;
    ssize_t nread;
    char *ptr;
    int buflen = 0;

    ptr=vptr;
    nleft=n;
    bzero((void *)vptr,1024);
    while(nleft > 0)
    {
        if((nread = read(uart_fd,ptr,nleft)) < 0)
        {
            if(errno == EINTR)//被信号中断
                nread = 0;
            else
                return -1;
        } else if(nread == 0)
        {
            break;
        }else
        {
            printf("%s--->%d\n",uart_buf_read,__LINE__);
           // printf("----------------------------\n");
          //  buflen =  strlen(vptr);
          //  vptr[buflen-2]='\0';
          //  printf("%d\n",buflen);
            ui->uart_recv_text->moveCursor(QTextCursor::End);
            ui->uart_recv_text->insertPlainText(uart_buf_read);
            bzero((void *)vptr,1024);
        }
        nleft -= nread;
        ptr += nread;
    }
    return (n-nleft);
}

int uart_open(int uart_fd,const char *pathname,Ui::uart *ui)
{
    //assert(pathname);

    char pathbuf[20] = {0};
    sprintf(pathbuf,"/dev/%s",pathname);
//	std::string path = pathname;
    /*打开串口*/
   //uart_fd = open(pathname,O_RDWR|O_NOCTTY|O_NDELAY);
 //  printf("-->%s---->%d\n",pathbuf,__LINE__);
    uart_fd = open(pathbuf,O_RDWR|O_NOCTTY|O_NDELAY);
    if(uart_fd == -1)
    {
        //perror("Open UART failed!");
        ui->uart_err_box->setText("open uart failed");
        return -1;
    }

    /*清除串口非阻塞标志*/
    if(fcntl(uart_fd,F_SETFL,0) < 0)
    {
        //fprintf(stderr,"fcntl failed!\n");
        ui->uart_err_box->setText("fcntl failed");
        return -1;
    }

    return uart_fd;
}

int uart_set(int uart_fd,int baude,int c_flow,int bits,int parity,int stop,Ui::uart *ui)
{
    struct termios options;

    pthread_t read_id;
    pthread_t write_id;

    memset(&options,0,sizeof(options));
    /*获取终端属性*/
    if(tcgetattr(uart_fd,&options) < 0)
    {
        //perror("tcgetattr error");
         ui->uart_err_box->setText("tcgetattr error");
        return -1;
    }


    /*设置输入输出波特率，两者保持一致*/
    switch(baude)
    {
        case 4800:
            cfsetispeed(&options,B4800);
            cfsetospeed(&options,B4800);
            break;
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 115200:
            cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            printf("115200+%d\n",__LINE__);
            break;
        default:
           // fprintf(stderr,"Unkown baude!\n");
        ui->uart_err_box->setText("unkown baude");
            return -1;
    }

    /*设置控制模式*/
    options.c_cflag |= CLOCAL;//保证程序不占用串口
    options.c_cflag |= CREAD;//保证程序可以从串口中读取数据

    /*设置数据流控制*/
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            printf("c_flow+%d\n",__LINE__);
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            //fprintf(stderr,"Unkown c_flow!\n");
        ui->uart_err_box->setText("unkonw flow");
            return -1;
    }

    /*设置数据位*/
    switch(bits)
    {
        case 5:

            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS8;
            printf("bits+%d\n",__LINE__);
            break;
        default:
           // fprintf(stderr,"Unkown bits!\n");
        ui->uart_err_box->setText("unkown bits");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 0:

            //printf("parity+%d\n",__LINE__);
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验(本句取反，取消奇偶检验)
            break;
        /*设为空格,即停止位为2位*/
        case 3:
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 1:
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
         //   options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 2:
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
          //  options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            //fprintf(stderr,"Unkown parity!\n");
        ui->uart_err_box->setText("unknow parity");
            return -1;
    }

    /*设置停止位*/
    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            printf("stop+%d\n",__LINE__);
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
           // fprintf(stderr,"Unkown stop!\n");
        ui->uart_err_box->setText("unknow stopbits");
            return -1;
    }

    /*设置输出模式为原始输出*/
    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/
    options.c_cc[VTIME] = 0;//设置超时时间
    options.c_cc[VMIN] = 0;//轮询读取数据

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(uart_fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(uart_fd,TCSANOW,&options) < 0)
    {
       ui->uart_err_box->setText("set uart failed");
        return -1;
    }

    if(!pthread_on)
    {

        if(  pthread_create(&read_id, NULL, pthread_read, (void *)ui ) != 0)
        {

            ui->uart_err_box->setText("fail to pthread_cteate_read");
            exit(1);
        }

        if(  pthread_create(&write_id, NULL, pthread_write, (void *)ui ) != 0)
        {

            ui->uart_err_box->setText("fail to pthread_cteate_write");
            exit(1);
        }
        pthread_on =1;
    }

    uart_on = 1;
    ui->uart_err_box->setText("uart_set is ok");
    return 0;
}

int uart_read(int fd,char *r_buf,size_t len,Ui::uart *ui)
{
    ssize_t cnt = 0;
    cnt = safe_read(fd,r_buf,len,ui);
    if(cnt == -1)
    {
           ui->uart_err_box->setText("read error");
           return -1;
    }
           return cnt;
}

int uart_write(int fd,const char *w_buf,size_t len,Ui::uart *ui)
{
    ssize_t cnt = 0;
    if(uart_send_en == 1)
    {
      cnt = safe_write(fd,w_buf,len,ui);
       printf(" cnt = safe_write(fd,w_buf,len,ui);433\n");
       if(cnt == -1)
      {
           ui->uart_err_box->setText("safe_write error");
           return -1;
      }

      return cnt;
    }
    return 0;
}

int uart_close(int uart_fd,Ui::uart *ui)
{

    close(uart_fd);
    uart_read_en=0;
    uart_send_en =0;
    uart_on = 0;
    uart_fd = -1;
    /*可以在这里做些清理工作*/
    ui->uart_err_box->setText("uart_file has been closed");
    return 0;
}

int uart_main(const char * file_uart,int baude,int c_flow,int bits,int  parity,int stop,Ui::uart *ui)
{
    uart_fd = uart_open(uart_fd,file_uart,ui);/*串口号/dev/ttySn,USB口号/dev/ttyUSBn*/
    if(uart_fd == -1)
    {
       ui->uart_err_box->setText("open failed");
        return -1;
    }

    if(uart_set(uart_fd,baude,c_flow,bits,parity,stop,ui) == -1)
    {
            ui->uart_err_box->setText("uart_set failed ");
            return -1;
    }
    return 0;

}



uart::uart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::uart)
{
    ui->setupUi(this);
    read_text = ui->uart_recv_text;
}

uart::~uart()
{
    delete ui;
}

void uart::on_uart_send_clicked()
{
    const  char *writebuf= ui->uart_send_text->toPlainText().toStdString().c_str();
    QString uart_sendway_str = ui->uart_send_way->currentText();   //发送方式
    //uart_send_time = uart_sendway_str.toInt()*1000;

    QString uart_settime_str = ui->uart_set_time->text();   //自动发送时间
    uart_send_time = uart_settime_str.toInt()*1000;
    printf("-->%s,-->%d\n", writebuf,uart_send_time);
    ui->uart_err_box->setText(ui->uart_send_text->toPlainText());
    if(ui->uart_send_text->toPlainText().toStdString().c_str()==NULL)
    {
        ui->uart_err_box->setText("the send area is empty");
        return ;
    }
    memset(uart_buf_write,0,sizeof(uart_buf_write));
    if(strcpy(uart_buf_write, writebuf) ==NULL)
    {
        ui->uart_err_box->setText("buf_write_cpy failed");
        return ;
    }

    if(!uart_on)
    {
        ui->uart_err_box->setText("please open uart_file");
        return ;
    }
    else
    {
        if(strcmp(uart_sendway_str.toStdString().c_str(),"auto")==0)
        {
            uart_send_auto = 1;
        }
        else
        {
            uart_send_auto = 0;
        }
    }

    uart_send_en =1;
}

void uart::on_uatr_on_clicked()
{
    QString start_stop_str = ui->uart_start_stop->currentText();          //uart开关控制项
    QString uart_file_str = ui->uart_file->currentText();                    //uar控制台设备文件
    QString uart_speed_str = ui->uart_speed->currentText();                //波特率选项
    QString uart_stopbit_str = ui->uart_stopbit->currentText();            //停止位
    QString uart_databit_str = ui->uart_data_bit->currentText();          //数据位选项
    QString uart_checkbit_str = ui->uart_check_bit->currentText();         // 检验位
    QString uart_flow_str = ui->uart_flow_con->currentText();                //流控制


    int flow_con = 0;  //将流控制转化为整形数，方便处理
    if(strcmp(uart_flow_str.toStdString().c_str(),"None") == 0)
    {
        flow_con= 0;
    }else if(strcmp(uart_flow_str.toStdString().c_str(),"soft") == 0)
    {
        flow_con = 1;
    }else
    {
        flow_con = 2;
    }


    if(strcmp(start_stop_str.toStdString().c_str(),"start")!=0)
    {

        ui->uart_err_box->setText("please choose start!!!");
        return ;
    }

    if (!uart_on)
    {
        uart_main(uart_file_str.toStdString().c_str(),
                  uart_speed_str.toInt(),
                  flow_con,
                  uart_databit_str.toInt(),
                  uart_checkbit_str.toInt(),
                  uart_stopbit_str.toInt(),ui);
    }
    else
    {

        ui->uart_err_box->setText("the file has been opened !!!");
        return ;

    }

}

void uart::on_uart_off_clicked()
{
    QString start_stop_str = ui->uart_start_stop->currentText();

    if(strcmp(start_stop_str.toStdString().c_str(),"stop")!=0)
    {

        ui->uart_err_box->setText("please choose stop!!!");
        return ;
    }
    uart_close(uart_fd,ui);

}

void uart::on_uart_recv_clicked()
{
    if(!uart_on)
    {

        ui->uart_err_box->setText("please open uart_file");
        return ;
    }
    else
    {
        uart_read_en = 1;
    }

}
