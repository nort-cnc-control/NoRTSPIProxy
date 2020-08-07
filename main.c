
/*
NoRT SPI Sender
Copyright (C) 2020  Vladislav Tsendrovskii

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
Program description:

This program is designed for transmitting lines to SPI, and receive frames from SPI and send them as line.

Frame format: hightbyte:lowbyte:string_without_'\n'

Lines are received and transmitted from/to TCP:8889

How to run:

nort_spi_proxy spidev 7

where 

spidev is /dev/spidev* and '7' is a gpio number for interrupt from device

*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <arpa/inet.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <string.h>
#include <gpiod.h>

int sendline(int sock, const char *buf, size_t len);

int spi_speed = 1000000;
int spi_bits  = 8;
int spi_delay = 0;

#define MAXLEN 120

uint8_t zero[MAXLEN];

int spi_transfer(int spidev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = spi_delay,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
    };

    int ret = ioctl(spidev, SPI_IOC_MESSAGE(1), &tr);
    return ret;
}

int spi_read(int spidev, uint8_t *rx, int len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)zero,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = spi_delay,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
    };

    int ret = ioctl(spidev, SPI_IOC_MESSAGE(1), &tr);
    return ret;
}


volatile int run;
int ctlsock;
int clientctlsock;

uint8_t resp_buf[MAXLEN];
int resp_pos;
int resp_len;
bool resp_frame = false;
uint8_t rx_buf[MAXLEN];
uint8_t tx_buf[MAXLEN];

struct gpiod_chip *chip;
struct gpiod_line *line;
struct gpiod_line_event event;

void handle_rx(const uint8_t *data, size_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        if (data[i] == 0xFF)
        {
            resp_frame = true;
        }
        else if (resp_frame)
        {
            resp_buf[resp_pos] = data[i];
            resp_pos++;
            if (resp_pos == 2)
            {
                resp_len = ((int)resp_buf[0]) << 8 | resp_buf[1];
            }
            if (resp_pos == resp_len + 2)
            {
                sendline(clientctlsock, resp_buf + 2, resp_len);
                resp_pos = 0;
                resp_len = 0;
                resp_frame = false;
            }
        }
    }
}

pthread_mutex_t spi_mutex;

void send_command_to_rt(int spidev, const char *buf, size_t len)
{
    pthread_mutex_lock(&spi_mutex);

    size_t msglen = 0;
  
    tx_buf[msglen++] = 0xFF; 
    tx_buf[msglen++] = (len >> 8) & 0xFF;
    tx_buf[msglen++] = len & 0xFF;

    memcpy(tx_buf + msglen, buf, len);
    msglen += len;
    printf("Send to SPI\n");

    spi_transfer(spidev, tx_buf, rx_buf, msglen);
    handle_rx(rx_buf, msglen);
    
    pthread_mutex_unlock(&spi_mutex);
}

void ask_new_message(int spidev)
{
    spi_read(spidev, rx_buf, 4);
    handle_rx(rx_buf, 4);
    if (resp_frame)
    {
        size_t len = resp_len - (resp_pos - 2);
        spi_read(spidev, rx_buf, len);
        handle_rx(rx_buf, len);
    }
}

bool has_new_messages(void)
{
    return !gpiod_line_get_value(line);
}

void ask_new_messages(int spidev)
{
    while (has_new_messages())
    {
        pthread_mutex_lock(&spi_mutex);
        ask_new_message(spidev);
        pthread_mutex_unlock(&spi_mutex);
    }
}

void *gpio_poll_cycle(void *arg)
{
    int spidev = *((int*)arg);
    ask_new_messages(spidev);
    while (run)
    {
        if (!has_new_messages())
            gpiod_line_event_wait(line, NULL);
        ask_new_messages(spidev);
    }

    return NULL;
}

int readline(int sock, char *buf)
{
    int i = 0;
    char c;
    do
    {
        int res = recv(sock, &c, 1, 0);
        if (res <= 0)
        {
            return -1;
        }
        if (c != '\n')
            buf[i++] = c;
    } while (c != '\n');
    buf[i++] = 0;
    return i-1;
}

int sendline(int sock, const char *buf, size_t len)
{
    if (sock == -1)
        return -1;
    const char cr = '\n';
    send(sock, buf, len, 0);
    send(sock, &cr, 1, 0);
    return len + 1;
}

int create_control(unsigned short port)
{
    struct sockaddr_in ctl_sockaddr;
    int ctlsock = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(ctlsock, SOL_SOCKET, SO_REUSEADDR, NULL, 0);

    ctl_sockaddr.sin_family = AF_INET;
    ctl_sockaddr.sin_port = htons(port);
    ctl_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(ctlsock, (struct sockaddr *)&ctl_sockaddr, sizeof(ctl_sockaddr)) < 0)
    {
        printf("Can not bind tcp\n");
        return -1;
    }
    return ctlsock;
}

int hex2dig(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';

    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;

    return -1;
}

int main(int argc, const char **argv)
{
    int port = 8889;
    int i;
    
    const char* spidev_name = "/dev/spidev0.0";
    int spidev;
    int gpio = 7;
    
    if (argc > 2)
    {
        spidev_name = argv[1];
        gpio = atoi(argv[2]);
    }

    // init spidev
    spidev = open(spidev_name, O_RDWR);
    if (spidev < 0)
    {
        return -1;
    }

    // Init GPIO
    int rv;
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip)
    {
        return -1;
    }
    line = gpiod_chip_get_line(chip, gpio);
    if (!line)
    {
        gpiod_chip_close(chip);
        return-1;
    }
    rv = gpiod_line_request_input(line, "foobar");
    if (rv)
    {
        gpiod_chip_close(chip);
        return -1;
    }
    gpiod_line_request_falling_edge_events(line, "foobar");

    // run gpio check thread
    run = 1;
    pthread_t thread;
    int res = pthread_create(&thread, NULL, gpio_poll_cycle, &spidev);

    // listen commands
    ctlsock = create_control(port);
    if (ctlsock < 0)
        return ctlsock;
    listen(ctlsock, 1);
    while (1)
    {
        char buf[1500];
        clientctlsock = accept(ctlsock, NULL, NULL);
        printf("Connect from client\n");
        while (1)
        {
            int len = readline(clientctlsock, buf);
            if (len < 0)
            {
                break;
            }
            printf("RECEIVE CTL: %.*s\n", len, buf);
            if (!strncmp(buf, "EXIT:", 5))
            {
                break;
            }
            else if (!strncmp(buf, "RT:", 3))
            {
                const char *cmd = buf + 3;
                send_command_to_rt(spidev, cmd, len-3);
            }
        }
        printf("Client disconnected\n");
        close(clientctlsock);
        clientctlsock = -1;
    }
    run = 0;
    gpiod_chip_close(chip);
    pthread_join(thread, NULL);
    close(ctlsock);
    return 0;
}

