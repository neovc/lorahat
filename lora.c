#include <sys/types.h>
#include <stdio.h>
#include <getopt.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>

/*
 *  -b serial baudrate
 *  -a lora airspeed
 *  -s lora hat tty
 *  -f lora frequence in Mhz
 *  -n lora net id
 *  -p tx power
 *  -z buffer size
 *  -k crypt key
 *  -v verbose, print rssi & other infos
 */

int bps = 9600, lora_airspeed = 9600, lora_freq = 433, lora_netid = 0, lora_power = 22, lora_buffersize = 1000, verbose_mode = 0, lora_addr = 0;
uint16_t lora_key = 0;
char lora_tty[50] = "/dev/ttyS0";
int lora_fd = -1;

#define M0 "22"
#define M1 "27"
#define AUX "4"

#define SX126X_UART_BAUDRATE_1200 0x00
#define SX126X_UART_BAUDRATE_2400 0x20
#define SX126X_UART_BAUDRATE_4800 0x40
#define SX126X_UART_BAUDRATE_9600 0x60
#define SX126X_UART_BAUDRATE_19200 0x80
#define SX126X_UART_BAUDRATE_38400 0xA0
#define SX126X_UART_BAUDRATE_57600 0xC0
#define SX126X_UART_BAUDRATE_115200 0xE0

#define SX126X_PACKAGE_SIZE_240_BYTE 0x00
#define SX126X_PACKAGE_SIZE_128_BYTE 0x40
#define SX126X_PACKAGE_SIZE_64_BYTE 0x80
#define SX126X_PACKAGE_SIZE_32_BYTE 0xC0

#define SX126X_POWER_22DBM 0x00
#define SX126X_POWER_17DBM 0x01
#define SX126X_POWER_13DBM 0x02
#define SX126X_POWER_10DBM 0x03

#define SX126X_AIRSPEED_1200 0x01
#define SX126X_AIRSPEED_2400 0x02
#define SX126X_AIRSPEED_4800 0x03
#define SX126X_AIRSPEED_9600 0x04
#define SX126X_AIRSPEED_19200 0x05
#define SX126X_AIRSPEED_38400 0x06
#define SX126X_AIRSPEED_62500 0x07

void
print_help(void)
{
	printf("usage: loraip [optargs]\r\n"
	       " -a airspeed, set lora airspeed, default is 9600\r\n"
	       " -A lora address, set lora address, default is 0\r\n"
	       " -b baudrate, set tty baudrate, default is 9600\r\n"
	       " -s tty, set lorahat serial tty, default is /dev/ttyS0\r\n"
	       " -f freq, set lora freq in Mhz, default is 433MHz\r\n"
	       " -n netid, set lora netid, default is 0\r\n"
	       " -p power, set lora tx power in dBm, default is 22dBm\r\n"
	       " -z size, set lorahat serial buffer size, default is 1000 Bytes\r\n"
	       " -k key, set lora crypt key, default is 0, don't crypt\r\n"
	       " -v, verbose mode\r\n"
	      );
	exit(0);
}

/**
 * Dumps a chunk of memory to the screen
 */
void
hex_dump(void *src, int len, int with_header)
{
	int i, j = 0, k;
	char text[17];

	text[16] = '\0';
	if (with_header)
		printf("%p : ", src);
	for (i = 0; i < len; i ++) {
		j ++;
		printf("%02X ", ((volatile unsigned char *)src)[i]);
		if (j == 8)
			printf(" ");
		if (j == 16) {
			j = 0;
			memcpy(text, &((char *)src)[i-15], 16);
			for (k = 0; k < 16; k ++) {
				if ((text[k] < 32) || (text[k] > 126)) {
					text[k] = '.';
				}
			}
			printf(" |");
			printf("%s", text);
			printf("|\n\r");
			if (with_header && (i < (len - 1))) {
				printf("%p : ", src + i + 1);
			}
		}
	}

	if (i % 16)
		printf("\r\n");
}

/* return 0 if OK
 * return 1 if FAILED
 */
int
set_nonblock(int fd)
{
	int flag = 1;
	struct linger ling = {0, 0};
	int r = 0;

	if (fd < 0) return 1;

	r = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
	r = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
	r = setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
	r = setsockopt(fd, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));

	flag = fcntl(fd, F_GETFL, 0);
	if (flag >= 0)
		fcntl(fd, F_SETFL, flag | O_NONBLOCK);
	return r;
}

/* return 0 if found.
 * return 1 if not found
 */
int
find_magic(const uint8_t *src, int len, uint16_t match, int *r)
{
	int i;
	uint8_t key[2];

	if (src == NULL || len < 2 || r == NULL) return 1;

	key[0] = (uint8_t)((match >> 8) & 0xff);
	key[1] = (uint8_t)(match & 0xff);

	for (i = 0; i < len - 1; i ++) {
		if (src[i] == key[0] && src[i + 1] == key[1]) {
			*r = i;
			return 0;
		}
	}

	return 1;
}

#define OUTPUT "out"
#define INPUT "in"
#define LOW "0"
#define HIGH "1"

/* return 0 if OK
 * return -1 if FAILED
 */
int
setup_pin(char *pin, char *mode)
{
	FILE *fp;
	char path[65] = "/sys/class/gpio/export";
	int k, r = 0;

	if (pin == NULL || pin[0] == '\0' || mode == NULL || mode[0] == '\0')
		return -1;

	fp = fopen(path, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't write %s: %s\r\n", path, strerror(errno));
		return -1;
	}

	k = strlen(pin);
	if (k != fwrite(pin, 1, k, fp)) {
		fprintf(stderr, "write %s to file %s failed: %s\r\n", pin, path, strerror(errno));
		fclose(fp);
		return -1;
	}
	fclose(fp);

	usleep(100000); /* sleep 0.1s before next operation */

	snprintf(path, 64, "/sys/class/gpio/gpio%s/direction", pin);

	fp = fopen(path, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't write %s: %s\r\n", path, strerror(errno));
		return -1;
	}

	k = strlen(mode);
	if (k != fwrite(mode, 1, k, fp)) {
		fprintf(stderr, "write %s to file %s failed: %s\r\n", mode, path, strerror(errno));
		r = -1;
	}
	fclose(fp);
	return r;
}

/* return 0 if OK
 * return -1 if FAILED
 */
int
write_pin(char *pin, char *value)
{
	char path[65];
	FILE *fp;
	int k, r = 0;

	if (pin == NULL || pin[0] == '\0' || value == NULL || value[0] == '\0')
		return -1;

	snprintf(path, 64, "/sys/class/gpio/gpio%s/value", pin);

	fp = fopen(path, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't write %s: %s\r\n", path, strerror(errno));
		return -1;
	}

	k = strlen(value);
	if (k != fwrite(value, 1, k, fp)) {
		fprintf(stderr, "write %s to file %s failed: %s\r\n", value, path, strerror(errno));
		r = -1;
	}
	fclose(fp);
	return r;
}

/* return 0 or 1 if OK
 * return -1 if read FAILED
 */
int
read_pin(char *pin)
{
	char path[65], c;
	FILE *fp;

	if (pin == NULL || pin[0] == '\0')
		return -1;

	snprintf(path, 64, "/sys/class/gpio/gpio%s/value", pin);

	fp = fopen(path, "r");
	if (fp == NULL) {
		fprintf(stderr, "can't read %s: %s\r\n", path, strerror(errno));
		return -1;
	}

	if (1 != fread(&c, 1, 1, fp)) {
		fprintf(stderr, "read from %s failed: %s\r\n", path, strerror(errno));
		c = '0' - 1;
	}
	fclose(fp);
	return (c - '0');
}

/* return 0 if OK
 * return -1 if FAILED
 */
int
release_pin(char *pin)
{
	FILE *fp;
	char path[65] = "/sys/class/gpio/unexport";
	int k, r = 0;

	if (pin == NULL || pin[0] == '\0')
		return -1;

	fp = fopen(path, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't write %s: %s\r\n", path, strerror(errno));
		return -1;
	}
	k = strlen(pin);
	if (k != fwrite(pin, 1, k, fp)) {
		fprintf(stderr, "write %s to file %s failed: %s\r\n", pin, path, strerror(errno));
		r = -1;
	}
	fclose(fp);
	return r;
}

/* return fd of opened ttys of baudrate with 8N1
 * return -1 if failed.
 */
int
open_tty(char *dev, int speed, int non_block)
{
	struct termios options;
	int fd, rate, flag;

	if (dev == NULL || dev[0] == '\0')
		return -1;

	fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		fprintf(stderr, "can't open %s: %s\r\n", dev, strerror(errno));
		return -1;
	}

	switch(speed) {
	case 2400:
		rate = B2400;
		break;
	case 4800:
		rate = B4800;
		break;
	case 9600:
		rate = B9600;
		break;
	case 19200:
		rate = B19200;
		break;
	case 38400:
		rate = B38400;
		break;
	case 57600:
		rate = B57600;
		break;
	case 115200:
		rate = B115200;
		break;
	case 460800:
		rate = B460800;
		break;
	case 500000:
		rate = B500000;
		break;
	case 921600:
		rate = B921600;
		break;
	case 1000000:
		rate = B1000000;
		break;
	case 1500000:
		rate = B1500000;
		break;
	case 2000000:
		rate = B2000000;
		break;
	case 2500000:
		rate = B2500000;
		break;
	case 3000000:
		rate = B3000000;
		break;
	default:
		rate = 9600;
		break;
	}

	if (-1 == tcgetattr(fd, &options)) {
		fprintf(stderr, "tcgetattr(#%d) failed, %s\r\n", fd, strerror(errno));
		close(fd);
		return -1;
	}
	cfsetispeed(&options, rate);
	cfsetospeed(&options, rate);

	options.c_cflag |= (CLOCAL | CREAD);
	/* 8N1 */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	/* non-blocking read */
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;

	if (-1 == tcsetattr(fd, TCSANOW, &options)) {
		fprintf(stderr, "tcsetattr(#%d) failed, %s\r\n", fd, strerror(errno));
		close(fd);
		return -1;
	}

	if (non_block) {
		/* set non-blocking tty fd */
		flag = fcntl(fd, F_GETFL, 0);
		if (flag >= 0)
			fcntl(fd, F_SETFL, flag | O_NONBLOCK);
	}

	/* Flush old transmissions */
	if (tcflush(fd, TCIOFLUSH) == -1)
		fprintf(stderr, "flushing serial port %s failed, %s\r\n", dev, strerror(errno));

	return fd;
}

/* return 0 if init OK
 * return -1 if init failed
 */
int
init_lorahat(int fd, int baud, int rx_freq, int rx_addr, int rx_netid, int tx_power, int airspeed, int buffersize, int cryptkey)
{
	uint8_t cfg_reg[] = {0xC2, 0x00, 0x09, 0x00, 0x00, 0x00, 0x62, 0x00, 0x12, 0x43, 0x00, 0x00}, buf[13];
	uint8_t rxfreq_off, baud_key, airspeed_key, power_key, buffersize_key;
	int r;

	if (fd < 0)
		return -1;

	if ((rx_freq <= 930) && (rx_freq >= 850)) {
		rxfreq_off = rx_freq - 850;
	} else if ((rx_freq <= 493) && (rx_freq >= 410)) {
		rxfreq_off = rx_freq - 410;
	} else {
		fprintf(stderr, "bad lora rx_freq %d, should be 410-493 or 850-930\r\n", rx_freq);
		return -1;
	}

	/* address is 0 - 65535
	 * under the same frequence, if set 65535, the node can receive
	 * messages from another node of address is 0 to 65534 and similarly,
	 * the address 0 to 65534 of node can receive messages while
	 * the another note of address is 65535 sends.
	 * otherwise two node must be same the address and frequence
	 */
	cfg_reg[3] = (rx_addr >> 8) & 0xff;
	cfg_reg[4] = rx_addr & 0xff;

	/* netid, 0 - 255*/
	cfg_reg[5] = rx_netid & 0xff;

	switch (baud) {
	case 1200:
		baud_key = SX126X_UART_BAUDRATE_1200;
		break;
	case 2400:
		baud_key = SX126X_UART_BAUDRATE_2400;
		break;
	case 4800:
		baud_key = SX126X_UART_BAUDRATE_4800;
		break;
	case 19200:
		baud_key = SX126X_UART_BAUDRATE_19200;
		break;
	case 38400:
		baud_key = SX126X_UART_BAUDRATE_38400;
		break;
	case 57600:
		baud_key = SX126X_UART_BAUDRATE_57600;
		break;
	case 115200:
		baud_key = SX126X_UART_BAUDRATE_115200;
		break;
	case 9600:
	default:
		baud_key = SX126X_UART_BAUDRATE_9600;
		break;
	}

	switch (airspeed) {
	case 1200:
		airspeed_key = SX126X_AIRSPEED_1200;
		break;
	case 2400:
		airspeed_key = SX126X_AIRSPEED_2400;
		break;
	case 4800:
		airspeed_key = SX126X_AIRSPEED_4800;
		break;
	case 19200:
		airspeed_key = SX126X_AIRSPEED_19200;
		break;
	case 38400:
		airspeed_key = SX126X_AIRSPEED_38400;
		break;
	case 62500:
		airspeed_key = SX126X_AIRSPEED_62500;
		break;
	case 9600:
	default:
		airspeed_key = SX126X_AIRSPEED_9600;
		break;
	}

	cfg_reg[6] = baud_key + airspeed_key;

	switch (buffersize) {
	case 128:
		buffersize_key = SX126X_PACKAGE_SIZE_128_BYTE;
		break;
	case 64:
		buffersize_key = SX126X_PACKAGE_SIZE_64_BYTE;
		break;
	case 32:
		buffersize_key = SX126X_PACKAGE_SIZE_32_BYTE;
		break;
	case 240:
	default:
		buffersize_key = SX126X_PACKAGE_SIZE_240_BYTE;
		break;
	}

	switch (tx_power) {
	case 17:
		power_key = SX126X_POWER_17DBM;
		break;
	case 13:
		power_key = SX126X_POWER_13DBM;
		break;
	case 10:
		power_key = SX126X_POWER_10DBM;
		break;
	case 22:
	default:
		power_key = SX126X_POWER_22DBM;
		break;
	}

	cfg_reg[7] = buffersize_key + power_key + 0x20;
	cfg_reg[8] = rxfreq_off;
	cfg_reg[9] = 0x43; /* disable rssi and don't relay */
	// cfg_reg[9] = 0x43 + 0x80; /* enable rssi and don't relay */
	cfg_reg[10] = (cryptkey >> 8) & 0xff;
	cfg_reg[11] = cryptkey & 0xff;

	/* let lorahat enter config mode */
	write_pin(M0, "0");
	write_pin(M1, "1");
	usleep(100000); /* sleep 0.1s */

	/* write to lorahat */
	if (verbose_mode) {
		printf("write 12 bytes to lorahat:\r\n");
		hex_dump(cfg_reg, 12, 0);
	}

	r = write(fd, cfg_reg, 12);
	if (r != 12) {
		/* write failed */
		write_pin(M1, "0");
		fprintf(stderr, "write #12 cfg register to fd #%d failed, return %d\r\n", fd, r);
		return -1;
	}

	usleep(200000); /* sleep 0.2s */
	r = read(fd, buf, 12);

	write_pin(M0, "0");
	write_pin(M1, "0");

	if (r < 12) {
		fprintf(stderr, "read #12 bytes from fd #%d failed, return %d\r\n", fd, r);
		return -1;
	}

	if (verbose_mode) {
		printf("read 12 bytes from lorahat:\r\n");
		hex_dump(buf, 12, 0);
	}
	return 0;
}

/* return 0 if OK
 * return -1 if FAILED
 */
int
write_lorahat(int fd, int rx_freq, int rx_addr, char *msg)
{
	char *p, *q;
	char tx_buf[512];
	int tx_freq, tx_addr, tx_len, r;

	if (fd < 0 || msg == NULL || msg[0] == '\0')
		return -1;

	p = strchr(msg, ',');
	if (p == NULL)
		return -1;
	p[0] = '\0';
	tx_addr = atoi(msg);
	p[0] = ',';
	p ++;
	q = strchr(p, ',');
	if (q == NULL)
		return -1;
	q[0] = '\0';
	tx_freq = atoi(p);
	q[0] = ',';
	q ++;
	tx_buf[0] = (tx_addr >> 8) & 0xff;
	tx_buf[1] = tx_addr & 0xff;

	if ((tx_freq <= 930) && (tx_freq >= 850)) {
		tx_buf[2] = tx_freq - 850;
	} else if ((tx_freq <= 493) && (tx_freq >= 410)) {
		tx_buf[2] = tx_freq - 410;
	} else {
		tx_buf[2] = 0;
	}

	tx_buf[3] = (rx_addr >> 8) & 0xff;
	tx_buf[4] = rx_addr & 0xff;

	if ((rx_freq <= 930) && (rx_freq >= 850)) {
		tx_buf[5] = lora_freq - 850;
	} else if ((rx_freq <= 493) && (rx_freq >= 410)) {
		tx_buf[5] = rx_freq - 410;
	} else {
		tx_buf[5] = 0;
	}

	tx_len = strlen(q);
	memcpy(tx_buf + 6, q, tx_len);
	tx_buf[6 + tx_len] = '\0';
	tx_len += 6;

	r = write(fd, tx_buf, tx_len);
	if (r != tx_len) {
		fprintf(stderr, "write #%d data to fd %d failed, return %d\r\n", tx_len, fd, r);
		return -1;
	}
	return 0;
}

int
main(int argc, char **argv)
{
	int c, pos = 0, r;
	uint8_t rx_buf[512];
	char msg[512];

	while ((c = getopt(argc, argv, "a:A:b:z:s:p:n:k:f:v")) != -1) {
		switch (c) {
			case 'v':
				verbose_mode = 1;
				break;
			case 'a':
				lora_airspeed = atoi(optarg);
				break;
			case 'A':
				lora_addr = atoi(optarg);
				break;
			case 'b':
				bps = atoi(optarg);
				break;
			case 'z':
				lora_buffersize = atoi(optarg);
				break;
			case 's':
				strncpy(lora_tty, optarg, 49);
				break;
			case 'p':
				lora_power = atoi(optarg);
				break;
			case 'n':
				lora_netid = atoi(optarg);
				break;
			case 'k':
				lora_key = atoi(optarg);
				break;
			case 'f':
				lora_freq = atoi(optarg);
				break;
			default:
				print_help();
				break;
		}
	}

	/* open tty */
	lora_fd = open_tty(lora_tty, bps, 0);
	if (lora_fd < 0) {
		fprintf(stderr, "fail to open lorahat serial device %s\r\n", lora_tty);
		return 1;
	}

	printf("open %s -> fd #%d\r\n", lora_tty, lora_fd);

	if (setup_pin(M0, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", M0, OUTPUT);
		close(lora_fd);
		return 1;
	}

	if (setup_pin(M1, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", M1, OUTPUT);
		release_pin(M0);
		close(lora_fd);
		return 1;
	}

	if (setup_pin(AUX, INPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", AUX, INPUT);
		release_pin(M0);
		release_pin(M1);
		close(lora_fd);
		return 1;
	}

	c = init_lorahat(lora_fd, bps, lora_freq, lora_addr, lora_netid, lora_power, lora_airspeed, lora_buffersize, lora_key);
	printf("config lorahat -> %s\r\n", c == 0?"OK":"FAILED");

	if (c == 0) {
		printf("try to rx at %dM, addr %d, netid %d\r\npress 's' and enter to send message\r\n", lora_freq, lora_addr, lora_netid);
		while (1) {
			/* try to read from loarhat */
			c = -1;
			if (ioctl(lora_fd, FIONREAD, &c)) {
				if (errno != EAGAIN && errno != EINTR) {
					fprintf(stderr, "ioctl(#%d, FIONREAD) failed, %s\r\n", lora_fd, strerror(errno));
					break;
				} else {
					c = -1;
				}
			} else {
				// if (c > 0) fprintf(stderr, "ioctl(#%d, FIONREAD) return %d -> OK\r\n", lora_fd, c);
			}

			if (c > 0) {
				r = read(lora_fd, rx_buf + pos, c);
				if (r < 0) {
					fprintf(stderr, "read from fd %d return %d, %s\r\n", lora_fd, r, strerror(errno));
					break;
				}
				pos += r;
				if (verbose_mode)
					hex_dump(rx_buf, pos, 1);
				if (pos > 3) {
					rx_buf[pos] = '\0';
					printf("rx message: address %d, netid %d, \"%s\"\r\n", rx_buf[0] + (rx_buf[1] << 8), rx_buf[2], (char *)(rx_buf + 3));
					pos = 0;
				}
			} else {
				/* try to read from stdin */
				if (0 == ioctl(0, FIONREAD, &c) && c > 0) {
					c = getchar();
					if (c == 'S' || c == 's') {
						printf("enter address,freq,msg(0,433,xxxx): ");
						if (scanf("%s", msg) > 0) {
							if (0 == write_lorahat(lora_fd, lora_freq, lora_addr, msg)) {
								printf("write %s to lorahat -> OK\r\n", msg);
							}
						}
						printf("press 's' and enter to send message\r\n");
					}
				}
			}

			usleep(500000); /* sleep 0.5s */
		}
	}
	release_pin(M0);
	release_pin(M1);
	release_pin(AUX);
	close(lora_fd);
	return 0;
}
