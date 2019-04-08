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

void *uartrecv_thread(void * arg)
{
    fd_set read_fds;
    int uart_fd = *((int *)arg);
    //printf("uart fd  : %d\r\n", uart_fd);
    int ret;
    int len;
    uint8_t buff[13];

    while(1){
        FD_ZERO(&read_fds);
        FD_SET(uart_fd, &read_fds);
        ret = select(uart_fd + 1, &read_fds, NULL, NULL, NULL);
        if (ret < 0) {
            perror("select");
            continue;
        } else if (ret == 0) {
            //timeout 
        } else {
            if (FD_ISSET(uart_fd, &read_fds)) {
                FD_CLR(uart_fd, &read_fds);
                //memset(&buff[len], 0, 13-len);
                len += read(uart_fd, &buff[len], 128);
                //printf("recv len = %d\r\n", len);
                if (len == 13) {
                    len = 0;
                    if (buff[5] == 0x63) {
                    
                    } else {
                        recv_flag = 1;
                        recv_number++;

                        printf("recv:");
                        for(int i = 0; i< 13; i++) {
                            printf(" %02x", buff[i]);
                        }
                        printf("\r\n");
                    }
                    
                    //printf("Identify:%d, Resp:%d, CMD:%d\r\n", buff[9], buff[5],buff[4]);
                    memset(buff, 0, 13);
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    int fd;
    int len;
    int pos = 0;
    char buffer[13];
    pthread_t tid;
    int delay = 0;
    uint16_t crc;

    for(int i=0;i<7;i++) {
        crc = CRC16_IBM(&cmd[i][3],  9);
        cmd[i][12] = crc & 0x00ff;
        cmd[i][13] = crc >> 8;
    }

    fd = open("/dev/tty.SLAB_USBtoUART", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return -1;
    }
    printf("fd=%d\r\n", fd);

    if ( uart_set(fd, 115200, 0, 8, 'O', 1) != 0)
        printf("uart set paramter error\r\n");

    pthread_create(&tid, NULL, uartrecv_thread, (void *)&fd);

    uint32_t count = 0;
    while(1)
    {
        //printf("send cmd%d\r\n", pos+1);

//        len = write(fd, cmd[0], 15);
//        if (len == 15) {
//            send_number++;
//            send_flag = 1;
//            recv_flag = 0;
//
//            if ((pos == 0) || (pos == 1))
//                response_delay = 1;
//            else
//                response_delay = 1;
//
//            sleep(response_delay);
//            
//        } else {
//            send_err_number++;
//        }
        
//        while(recv_flag == 0) {
//            sleep(1);
//            delay++;
//            if (delay > 5) {
//                delay = 0;
//                break;
//            }
//        }
//
//        sleep(2);

        //pos++;
        //if (pos >= 7) 
        //    pos = 0;

        sleep(1);
        count++;
        //printf("send:%d\trecv:%d\tsend err:%d\r\n", send_number, recv_number, send_err_number);
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
    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN] = 1;//最少读取一个字符

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
