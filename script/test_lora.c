#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>     
#include <termios.h>    
#include <errno.h>  
#include <pthread.h>
#include <time.h>

uint8_t cmd[][15] = {
//    {0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x01, 0, 0, 0, 1, 0x08, 0x02, 0x5a},
//    {0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x02, 0, 0, 0, 2, 0x0c, 0x03, 0x5a},
    {0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x03, 0, 0, 0, 3, 0xf0, 0x03, 0x5a},
    //{0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x04, 0, 0, 0, 4, 0x04, 0x01, 0x5a},
    //{0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x05, 0, 0, 0, 5, 0xf8, 0x01, 0x5a},
    //{0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x06, 0, 0, 0, 6, 0xfc, 0x00, 0x5a},
    //{0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x07, 0, 0, 0, 7, 0xba, 0xc0, 0x5a},
    //{0x00, 0x08, 0x1b, 0xa5, 0x0c, 0x08, 0x00, 0x08, 0, 0, 0, 8, 0xae, 0xc5, 0x5a},
};

int send_number = 0;
int recv_number = 0;
int send_flag = 0;
int recv_flag = 0;
int send_err_number = 0;
int response_delay = 0;

int uart_set(int fd,int baude,int c_flow,int bits,char parity,int stop);
uint16_t CRC16_IBM(uint8_t *puchMsg, uint8_t usDataLen);
static void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf);
static void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf);

#define RECV_LEN        13

int main(int argc, char *argv[])
{
    int fd;
    int len = 0;
    uint16_t crc;
    fd_set read_fds;
    struct tm *tm;
    time_t seconds;
    uint32_t count = 0;
    uint8_t buff[13];
    int ret;
    struct timeval tv;
    uint8_t msglen = 0;
    int baud;
    uint8_t databit;
    char parity;
    uint8_t stopbit;
    int select_delay;

    if (argc < 2) {
        printf("./func [uart name] [msg len]\r\n");
        printf("example:./func /dev/tty0 16");
        return 0;
    }

    for(int i=0;i<7;i++) {
        crc = CRC16_IBM(&cmd[i][3],  9);
        cmd[i][12] = crc & 0x00ff;
        cmd[i][13] = crc >> 8;
    }

    printf("argv[1]:%s\r\n", argv[1]);
    printf("argv[2]:%d\r\n", atoi(argv[2]));
    printf("argv[3]:%d\r\n", atoi(argv[3]));    //baud
    printf("argv[4]:%d\r\n", atoi(argv[4]));    //data bits
    printf("argv[5]:%s\r\n", argv[5]);          //parity
    printf("argv[6]:%d\r\n", atoi(argv[6]));    //stop bit
    printf("argv[7]:%d\r\n", atoi(argv[7]));    //delay

    fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    msglen = atoi(argv[2]);
    baud = atoi(argv[3]);
    databit = atoi(argv[4]);
    stopbit = atoi(argv[6]);
    select_delay = atoi(argv[7]);
    if (strcmp(argv[5], "O") == 0)
        parity = 'O';
    else if (strcmp(argv[5], "N") == 0)
        parity = 'N';
    else if (strcmp(argv[5], "E") == 0)
        parity = 'E';
    else
        parity = 'X';

    if (uart_set(fd, baud, 0, databit, parity, stopbit) != 0)
        printf("uart set paramter error\r\n");

    while(1)
    {

        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        tv.tv_sec = select_delay;
        tv.tv_usec = 500000;
        ret = select(fd + 1, &read_fds, NULL, NULL, &tv);

        if (ret < 0) {
            printf("select error\r\n");
            continue;
        } else if (ret == 0) {
            
        } else {
            if (FD_ISSET(fd, &read_fds)) {
                FD_CLR(fd, &read_fds);
                len += read(fd, buff+len, msglen - len);
                if (len == msglen) {
                    len = 0;
                    recv_number++;
                    
                    seconds = time(NULL);
                    tm = localtime(&seconds);
                    printf("[time:%02d:%02d:%02d] ***** ", tm->tm_hour, tm->tm_min, tm->tm_sec);
                    
                    for(int i=0;i<msglen;i++)
                        printf(" %02x", buff[i]);
                    printf("\r\n");
                } else {
                    continue;
                }    
            }
        }
        count++;
        printf("times:%d count:%d\r\n", count, recv_number);
    }
}

static void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{
    uint8_t i;
    uint8_t tmp = 0;
    for(i=0;i< 8;i++)
    {
        if(srcBuf[0] & (1 << i)) //将数据颠倒 0-7，1-6，2-5，3-4互换位置
            tmp |= 1 << (7-i);
    }
    dBuf[0] = tmp;
}

static void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf)
{
    uint8_t i;
    uint16_t tmp = 0;

    for(i=0;i< 16;i++)
    {
        if(srcBuf[0]& (1 << i))
            tmp|=1<<(15 - i);
    }
    dBuf[0] = tmp;
}

uint16_t CRC16_IBM(uint8_t *puchMsg, uint8_t usDataLen)
{
    uint16_t wCRCin = 0x0000;
    uint16_t wCPoly = 0x8005;
    uint8_t wChar = 0;
    uint8_t i;

    while (usDataLen--)   
    {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
            if(wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }
    InvertUint16(&wCRCin,&wCRCin);

    return wCRCin;
}

int uart_set(int fd,int baude,int c_flow,int bits,char parity,int stop)
{
    struct termios options;

    /*获取终端属性*/
    if(tcgetattr(fd,&options) < 0)
    {
        perror("tcgetattr error");
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
        case 57600:
            cfsetispeed(&options,B57600);
            cfsetospeed(&options,B57600);
            break;
        case 115200:
            cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
        default:
            fprintf(stderr,"Unkown baude!\n");
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
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    /*设置数据位*/
    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
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
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
            /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
            /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
            /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    /*设置停止位*/
    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
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
    options.c_cc[VTIME] = 100;//可以在select中设置
    options.c_cc[VMIN] = 13;//最少读取一个字符

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;

}
