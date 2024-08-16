#include "../inc/uart.h"

/**
 * libtty_setcustombaudrate - set baud rate of tty device
 * @fd: device handle
 * @speed: baud rate to set
 *
 * The function return 0 if success, or -1 if fail.
 */
int ret;
int libtty_setcustombaudrate(int fd, int baudrate)
{
    struct termios2 tio;

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    return 0;
}

/**
 * libtty_setopt - config tty device
 * @fd: device handle
 * @speed: baud rate to set
 * @databits: data bits to set
 * @stopbits: stop bits to set
 * @parity: parity to set
 * @hardflow: hardflow to set
 *
 * The function return 0 if success, or -1 if fail.
 */
int libtty_setopt(int fd, int speed, int databits, int stopbits, char parity, char hardflow)
{
    struct termios newtio;
    struct termios oldtio;
    int i;

    memset(&newtio, 0, sizeof(newtio));

    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;

    /* set data bits */
    switch (databits)
    {
    case 5:
        newtio.c_cflag |= CS5;
        break;
    case 6:
        newtio.c_cflag |= CS6;
        break;
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "unsupported data size\n");
        return -1;
    }

    /* set parity */
    switch (parity)
    {
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB; /* Clear parity enable */
        newtio.c_iflag &= ~CSTOPB; /* Disable input parity check */
        break;
    case 'o':
    case 'O':
        newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even */
        newtio.c_iflag |= INPCK;             /* Enable input parity check */
        break;
    case 'e':
    case 'E':
        newtio.c_cflag |= PARENB;  /* Enable parity */
        newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
        newtio.c_iflag |= INPCK;   /* Enable input parity check */
        break;
    case 'm':
    case 'M':
        newtio.c_cflag |= PARENB; /* Enable parity */
        newtio.c_cflag |= CMSPAR; /* Stick parity instead */
        newtio.c_cflag |= PARODD; /* Even parity instead of odd */
        newtio.c_iflag |= INPCK;  /* Enable input parity check */
        break;
    case 's':
    case 'S':
        newtio.c_cflag |= PARENB;  /* Enable parity */
        newtio.c_cflag |= CMSPAR;  /* Stick parity instead */
        newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
        newtio.c_iflag |= INPCK;   /* Enable input parity check */
        break;
    default:
        fprintf(stderr, "unsupported parity\n");
        return -1;
    }

    /* set stop bits */
    switch (stopbits)
    {
    case 1:
        newtio.c_cflag &= ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |= CSTOPB;
        break;
    default:
        tcsetattr(fd, TCSANOW, &newtio);
        perror("unsupported stop bits\n");
        return -1;
    }

    if (hardflow)
        newtio.c_cflag |= CRTSCTS;
    else
        newtio.c_cflag &= ~CRTSCTS;

    newtio.c_cc[VTIME] = 0; /* Time-out value (tenths of a second) [!ICANON]. */
    newtio.c_cc[VMIN] = 0;  /* Minimum number of bytes read at once [!ICANON]. */
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) != 0)
    {
        perror("tcsetattr");
        return -1;
    }

    // /* set tty speed */
    // if (libtty_setcustombaudrate(fd, speed) != 0)
    // {
    //     perror("setbaudrate");
    //     return -1;
    // }

    return 0;
}

/**
 * libtty_open - open tty device
 * @devname: the device name to open
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
int libtty_open(const char *devname)
{
    int fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
    int flags = 0;

    if (fd < 0)
    {
        perror("open device failed");
        return -1;
    }

    flags = fcntl(fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) < 0)
    {
        printf("fcntl failed.\n");
        return -1;
    }

    if (isatty(fd) == 0)
    {
        printf("not tty device.\n");
        return -1;
    }
    else
        printf("tty device test ok.\n");

    return fd;
}

/**
 * libtty_sendbreak - uart send break
 * @fd: file descriptor of tty device
 *
 * Description:
 *  tcsendbreak() transmits a continuous stream of zero-valued bits for a specific duration, if the terminal
 *  is using asynchronous serial data transmission. If duration is zero, it transmits zero-valued bits for
 *  at least 0.25 seconds, and not more that 0.5 seconds. If duration is not zero, it sends zero-valued bits
 *  for some implementation-defined length of time.
 *
 *  If the terminal is not using asynchronous serial data transmission, tcsendbreak() returns without taking
 *  any action.
 */
int libtty_sendbreak(int fd)
{
    return tcsendbreak(fd, 0);
}

/**
 * libtty_write - write data to uart
 * @fd: file descriptor of tty device
 *
 * The function return the number of bytes written if success, others if fail.
 */
// static int libtty_write(int fd, float *data) //
// {
//     int nwrite;                       // count
//     u_char buf[64] = {0xff, 0xfe, 1}; // start

//     memset(buf + 3, 0x00, 61);                // refrash the buf but [0],[1]
//     memcpy(buf + 3, data, 3 * sizeof(float)); // you can put the 0xfffe and 0xa0d in you data too

//     *(buf + 15) = 0xa;
//     *(buf + 16) = 0xd;

//     nwrite = write(fd, buf, 17);
//     printf("wrote %d bytesa lready.\n", nwrite);

//     return nwrite;
// }

int libtty_write(int fd, __s16 *data, __u8 buff)
{
    int nwrite;           // count
    u_char buf[64] = {0}; // start
    buf[0] = 0xff;
    buf[1] = 0xfe;
    buf[2] = buff;
    memset(buf + 3, 0x00, 60);                // refrash the buf but [0],[1]
    memcpy(buf + 3, data, 3 * sizeof(__s16)); // you can put the 0xfffe and 0xa0d in you data too
                                              // cout<<*data<<*(data+1)<<*(data+2)<<endl;
    for(int i = 3;i < 9; i++)
        buf[9] += buf[i];
    *(buf + 10) = 0x0a;
    *(buf + 11) = 0x0d;
  

    nwrite = write(fd, buf, 12);
    sleep(0.01);
    printf("wrote %d bytesa lready.\n", nwrite);

    return nwrite;
}

int libtty_Write(int fd, std::vector<float> data, int buff, int count)
{
    int nwrite;           // count
    u_char buf[64] = {0}; // start
    buf[0] = 0xff;
    buf[1] = 0xfe;
    buf[2] = buff;

    memset(buf + 3, 0x00, 61);                                     // refrash the buf but [0],[1]
    memcpy(buf + 3, data.data(), (count * 2 + 1) * sizeof(float)); // you can put the 0xfffe and 0xa0d in you data too
                                                                   // cout<<*data<<*(data+1)<<*(data+2)<<endl;

    *(buf + (count * 2 + 1) * 4 + 3) = 0x0a;
    *(buf + (count * 2 + 1) * 4 + 4) = 0x0d;

    nwrite = write(fd, buf, 17);
    sleep(0.01);
    // printf("wrote %d bytesa lready.\n", nwrite);

    return nwrite;
}

void uart_init()
{
    signal(SIGINT, sig_handler);

    fd = libtty_open(device);
    printf("%d", fd);
    if (fd < 0)
    {
        printf("libtty_open: %s error.\n", device);
        exit(0);
    }

    ret = libtty_setopt(fd, speed, 8, 1, 'n', hardflow);
    if (ret != 0)
    {
        printf("libtty_setopt error.\n");
        exit(0);
    }
}
// static int libtty_write(int fd, float *data) //
// {
//   int nwrite;                       // count
//   u_char buf[64] = {0}; // start
//   buf[0] = 0xff;
//   buf[1] = 0xfe;
//   buf[2] = 0x02;
//   int32_t data_s32;
//   memset(&buf[3], 0x00, 61);                // refrash the buf but [0],[1]

//   for(int i = 0; i < 3;i++)
//   {
//     data_s32 = (int32_t)data[i];
//     memcpy(&buf[3], &data[i], sizeof(float)); // you can put the 0xfffe and 0xa0d in you data too

//   }

//   *(buf + 15) = 0xa;
//   *(buf + 16) = 0xd;

//   nwrite = write(fd, buf, 17);
//   printf("wrote %d bytesa lready.\n", nwrite);

//   return nwrite;
// }

static void sig_handler(int signo)
{
    printf("capture sign no:%d\n", signo);
    if (fp != NULL)
    {
        fflush(fp);
        fsync(fileno(fp));
        fclose(fp);
    }
    exit(0);
}

//   if ((width[i] * height[i]) > (width[i + 1] * height[i + 1]))
