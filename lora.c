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
#include <pthread.h>

#include <event.h> /* libevent */

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

int bps = 115200, lora_airspeed = 38400, lora_freq = 433, lora_netid = 0, lora_power = 22, lora_buffersize = 1000, verbose_mode = 0, lora_addr = 0;
uint16_t lora_key = 0;
char lora_tty[50] = "/dev/ttyS0";
int lora_fd = -1;

struct event_base *ev_base = NULL;
struct event lora_event;

uint8_t lora_rbuf[512];
int lora_rpos = 0, lora_rsize = 512;

#define M0 "22"
#define M1 "27"
#define AUX "4"

#define LORA_TEXT 0
#define LORA_BINARY 1
#define LORA_FILE 2
#define LORA_MAGIC 0xEB90

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
	       " -a airspeed, set lora airspeed, default is 38400\r\n"
	       " -A lora address, set lora address, default is 0\r\n"
	       " -b baudrate, set tty baudrate, default is 115200\r\n"
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

	fd = open(dev, O_RDWR | O_NOCTTY /* | O_NONBLOCK */);

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
	write_pin(M0, LOW);
	write_pin(M1, HIGH);
	usleep(100000); /* sleep 0.1s */

	/* write to lorahat */
	if (verbose_mode) {
		printf("write 12 bytes to lorahat:\r\n");
		hex_dump(cfg_reg, 12, 0);
	}

	r = write(fd, cfg_reg, 12);
	if (r != 12) {
		/* write failed */
		write_pin(M1, LOW);
		fprintf(stderr, "write #12 cfg register to fd #%d failed, return %d\r\n", fd, r);
		return -1;
	}

	usleep(200000); /* sleep 0.2s */
	r = read(fd, buf, 12);

	write_pin(M0, LOW);
	write_pin(M1, LOW);

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

uint8_t
sumofbuffer(uint8_t *src, int len)
{
	uint8_t c = 0;
	int i;

	if (src == NULL || len <= 0)
		return 0;

	for (i = 0; i < len; i ++)
		c += src[i];

	return c;
}

#if 0
/* return 0 if OK
 * return -1 if FAILED
 */
int
write_lorahat(int fd, int rx_freq, int rx_addr, char *msg, int len)
{
	char *p, *q;
	char tx_buf[512];
	int tx_freq, tx_addr, tx_len, r;

	if (fd < 0 || msg == NULL || msg[0] == '\0' || len <= 0)
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
		tx_buf[5] = rx_freq - 850;
	} else if ((rx_freq <= 493) && (rx_freq >= 410)) {
		tx_buf[5] = rx_freq - 410;
	} else {
		tx_buf[5] = 0;
	}

	tx_len = (q - msg); /* remove 0,433, header */
	tx_len = len - tx_len;
	tx_buf[6] = tx_len & 0xff; /* prepend length before payload message */
	memcpy(tx_buf + 7, q, tx_len);
	tx_buf[7 + tx_len] = '\0';
	tx_len += 7;

	r = write(fd, tx_buf, tx_len);
	if (r != tx_len) {
		fprintf(stderr, "write #%d data to fd %d failed, return %d\r\n", tx_len, fd, r);
		return -1;
	}
	return 0;
}
#endif

/* return -2 if EOF
 * return -1 if failed
 * return 0 if there has no data to read at this time
 * return > 0 if success
 *
 * if max_buf_len <= 0, buffer_size is unlimited
 */
int
read_buffer(int fd, uint8_t *b, int max_buf_len)
{
	int to_read = 0, r;

	if (fd <= 0 || b == NULL) return -1;

	if (ioctl(fd, FIONREAD, &to_read)) {
		if (errno != EAGAIN && errno != EINTR) return -1;
		else return 0;
	}

	if (to_read == 0) to_read = 1;
	if ((max_buf_len > 0) && (to_read > max_buf_len)) return -3;

	r = read(fd, b, to_read);
	if ((r == -1 && (errno != EINTR && errno != EAGAIN)) || r == 0) {
		/* connection closed or reset */
		if (r == 0) return -2; /* EOF */
		return -1;
	}

	return r;
}

/* return -2 if EOF
 * return -1 if failed
 * return 0 if there has no data to read at this time
 * return > 0 if success
 */
int
read_tty(int fd, uint8_t *b, int len)
{
	int r;

	if (fd <= 0 || b == NULL || len <= 0) return -1;

	if (len > 8)
		len = 8;
	r = read(fd, b, len);
	if ((r == -1 && (errno != EINTR && errno != EAGAIN)) || r == 0) {
		/* connection closed or reset */
		if (r == 0) return -2; /* EOF */
		return -1;
	}

	return r;
}

/* return 0 if OK
 * return -1 if FAILED
 */
int
evwrite_lorahat(int fd, int rx_freq, int rx_addr, int tx_freq, int tx_addr, uint8_t *payload, int len, uint8_t type)
{
	uint8_t tx_buf[512];
	int r, pos = 0;

	if (fd < 0 || payload == NULL || len <= 0 || len > 250)
		return -1;

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
		tx_buf[5] = rx_freq - 850;
	} else if ((rx_freq <= 493) && (rx_freq >= 410)) {
		tx_buf[5] = rx_freq - 410;
	} else {
		tx_buf[5] = 0;
	}

	/* add LORA MAGIC HERE */
	tx_buf[6] = (LORA_MAGIC >> 8) & 0xff;
	tx_buf[7] = LORA_MAGIC & 0xff;

	tx_buf[8] = (len & 0xff) + 2; /* prepend length before payload message */
	tx_buf[9] = type;
	memcpy(tx_buf + 10, payload, len);
	/* append sum of buffer */
	tx_buf[10 + len] = sumofbuffer(payload, len) + type;
	tx_buf[11 + len] = '\0';
	len += 11;

	/* we need to write all data to lorahat here */
	while (pos < len) {
		r = write(fd, tx_buf + pos, len - pos);
		if (r > 0) {
			pos += r;
		} else if ((r == -1 ) && (errno != EINTR) && (errno != EAGAIN)) {
			fprintf(stderr, "write #%d data to fd %d failed, return %d, %s\r\n", len - pos, fd, r, strerror(errno));
			return -1;
		}
	}
	return 0;
}

struct filecmd {
	char name[8]; /* 8 chars filename, just for test */
	int pos;
};

/* return 0 if OK
 * return -1 if FAILED
 */
int
rxfile(uint8_t *payload, int len)
{
	char name[9];
	int pos;
	FILE *fp;

	if (payload == NULL || len <= sizeof(struct filecmd))
		return -1;

	memcpy(name, payload, 8);
	name[8] = '\0';
	memcpy(&pos, payload + 8, sizeof(pos));

	if (pos == 0)
		fp = fopen(name, "wb"); /* overwrite file */
	else
		fp = fopen(name, "ab"); /* append file, don't overwrite file */
	if (fp == NULL) {
		fprintf(stderr, "can't write to file %s, %s\r\n", name, strerror(errno));
		return -1;
	}

	fseek(fp, pos, SEEK_SET);

	len -= sizeof(struct filecmd);
	if (len != fwrite(payload + sizeof(struct filecmd), 1, len, fp)) {
		fprintf(stderr, "write #%d data at pos %d to file %s failed, %s\r\n", len, pos, name, strerror(errno));
		fclose(fp);
		return -1;
	}
	fflush(fp);
	fclose(fp);
	printf("write #%d data at pos %d to file %s -> OK\r\n", len, pos, name);
	return 0;
}

#define BLOCKSIZE 200

/* return 0 if SEND OK
 * return -1 if SEND FAILED
 */
int
sendfile_lorahat(int fd, int rx_freq, int rx_addr, int tx_freq, int tx_addr, char *file)
{
	FILE *fp;
	uint8_t payload[512];
	int pos = 0, size, r = 0, len, bps, k;
	char name[9];
	time_t start, end;
	struct filecmd c;

	if (file == NULL || file[0] == '\0')
		return -1;

	fp = fopen(file, "rb");
	if (fp == NULL) {
		fprintf(stderr, "can't open %s, %s\r\n", file, strerror(errno));
		return -1;
	}

	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	start = time(NULL);
	snprintf(name, 9, "%d", (int) start);
	while (pos < size) {
		if ((size - pos) > BLOCKSIZE)
			len = BLOCKSIZE;
		else
			len = size - pos;

		if (len != fread(payload + sizeof(c), 1, len, fp)) {
			fprintf(stderr, "read #%d from %s at pos %d failed, %s\r\n", len, file, pos, strerror(errno));
			r = -1;
			break;
		}
		c.pos = pos;
		memcpy(c.name, name, 8);
		memcpy(payload, &c, sizeof(c));
		while (read_pin(AUX) == 0) {
			usleep(100); /* sleep 0.1ms before AUX goes to high */
		}
		if (-1 == evwrite_lorahat(fd, rx_freq, rx_addr, tx_freq, tx_addr, payload, len + sizeof(c), LORA_FILE)) {
			fprintf(stderr, "write to lorahat at #%d failed\r\n", pos);
			r = -1;
			break;
		}
		pos += len;
		k = 0;
		/* AUX -> LOW indicates beginning of LORA TX
		 *  ... LORA TXing ...
		 * AUX -> HIGH indicated end of LORA TX
		 * wait for 1ms make sure TX is stopped
		 */
		while (read_pin(AUX) == 1) {
			usleep(1000); /* sleep 1ms before AUX goes to low */
			k += 1;
		}
		while (read_pin(AUX) == 0) {
			usleep(1000); /* sleep 1ms before AUX goes to HIGH */
			k += 1;
		}
		usleep(1000);
		printf("send #%d at pos %d data to lorahat -> OK, %dms\r\n", len, pos, k);
	}

	end = time(NULL);
	if (end == start) {
		bps = pos * 8;
	} else {
		bps = pos * 8 / (end - start);
	}
	printf("sending %d/%d data to lora takes %d seconds, %dbps\r\n", pos, size, (int) (end - start), bps);
	fclose(fp);
	return r;
}

void
handle_lora(const int fd, short which, void *arg)
{
	int r, last, start;
	uint8_t calc_sum, sum, type, len;

	if (which & EV_READ) {
		r = read_tty(fd, lora_rbuf + lora_rpos, lora_rsize - lora_rpos);
		if (r < 0)
			return;

		lora_rpos += r;
		if (lora_rpos > 7) {
			/* to find LORAMAGIC 0xeb90 */
			lora_rbuf[lora_rpos] = '\0';
			r = find_magic(lora_rbuf, lora_rpos, LORA_MAGIC, &start);
			if (r == 1) {
				/* NO LORA_MAGIC FOUND */
				return;
			}

			if (start < 3) {
				/* not enough prefix for full message */
				/* skip this magic word */
				memmove (lora_rbuf, lora_rbuf + start + 2, lora_rpos - start - 2);
				lora_rpos -= start + 2;
				return;
			}

			if (start > 3) {
				/* move data to pos 0 */
				memmove (lora_rbuf, lora_rbuf + start - 3, lora_rpos - start + 3);
				lora_rpos -= start - 3;
			}

			if ((lora_rpos < 8) || (lora_rpos < (lora_rbuf[5] + 6))) {
				/* not full message */
				return;
			}

			/* [ADDR HIGH][ADDR LOW][NETID][0xEB][0x90][LEN][TYPE][PAYLOAD][SUM]
			 * LEN = SIZEOF(PAYLOAD) + 1 + 1
			 */
			if (verbose_mode)
				hex_dump(lora_rbuf, lora_rpos, 1);
			len = lora_rbuf[5];
			type = lora_rbuf[6];
			calc_sum = sumofbuffer(lora_rbuf + 6, len - 1);
			last = len + 6 - 1;
			sum = lora_rbuf[last];
			lora_rbuf[last] = '\0'; /* clear sum of buffer */

			if (calc_sum != sum) {
				printf("lora msg mismatched CALC 0x%x != DATA 0x%x\r\n", calc_sum, sum);
			} else if (type == LORA_TEXT) {
				printf("rx txt msg: address %d, netid %d, len %d, \"%s\"\r\n",
					((lora_rbuf[0] << 8) + lora_rbuf[1]), lora_rbuf[2], len - 2,
					(char *)(lora_rbuf + 7));
			} else if (type == LORA_BINARY) {
				printf("rx binary msg:\r\n");
				hex_dump(lora_rbuf + 7, len - 2, 0);
			} else if (type == LORA_FILE) {
				rxfile(lora_rbuf + 7, len - 2);
			}

			lora_rpos -= 6 + len;
			if (lora_rpos > 0) {
				/* keep unprocessed data */
				memmove (lora_rbuf, lora_rbuf + 6 + len, lora_rpos);
			}
		}
	}
}

void *
send_handler(void *arg)
{
	char msg[512];
	char *p, *q;
	int tx_freq, tx_addr, r;

	while (1) {
		printf("enter '[s|t]0,433,xxxx' to send message\r\n");
		if (NULL != fgets(msg, 512, stdin)) {
			/* parse address,freq string first */
			r = strlen(msg) - 1;
			if (r >= 0 && msg[r] == '\n')
				msg[r] = '\0';

			p = strchr(msg + 1, ',');
			if (p == NULL)
				continue;
			p[0] = '\0';
			tx_addr = atoi(msg + 1);
			p[0] = ',';
			p ++;
			q = strchr(p, ',');
			if (q == NULL)
				continue;
			q[0] = '\0';
			tx_freq = atoi(p);
			q[0] = ',';
			q ++;

			if (msg[0] == 's' || msg[0] == 'S') {
				/* send lora txt message */
				if (0 == evwrite_lorahat(lora_fd, lora_freq, lora_addr, tx_freq, tx_addr, (uint8_t *) q, strlen(q), LORA_TEXT))
					printf("send \"%s\" to lora #%d / %dMhz -> OK\r\n", q, tx_addr, tx_freq);
			} else if (msg[0] == 't' || msg[0] == 'T') {
				sendfile_lorahat(lora_fd, lora_freq, lora_addr, tx_freq, tx_addr, (char *) q);
			} else {
				printf("bad cmd %c\r\n", msg[0]);
			}
		}
	}
}

int
main(int argc, char **argv)
{
	int c, default_bps = 9600;
	pthread_t send_task;

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

	if (setup_pin(M0, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", M0, OUTPUT);
		return 1;
	}

	if (setup_pin(M1, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", M1, OUTPUT);
		release_pin(M0);
		return 1;
	}

	if (setup_pin(AUX, INPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\r\n", AUX, INPUT);
		release_pin(M0);
		release_pin(M1);
		return 1;
	}

	/* put lorahat in deepsleep mode */
	write_pin(M0, HIGH);
	write_pin(M1, HIGH);
	usleep(100000); /* sleep 0.1s */

	/* to reset lorahat settings */
	write_pin(M0, LOW);
	write_pin(M1, LOW);
	usleep(200000); /* sleep 0.2s */

	/* open lorahat's tty */
	lora_fd = open_tty(lora_tty, default_bps, 1);
	if (lora_fd < 0) {
		fprintf(stderr, "fail to open lorahat serial device %s\r\n", lora_tty);
		release_pin(M0);
		release_pin(M1);
		return 1;
	}

	printf("open %s - default #%d bps -> fd #%d\r\n", lora_tty, default_bps, lora_fd);

	c = init_lorahat(lora_fd, bps, lora_freq, lora_addr, lora_netid, lora_power, lora_airspeed, lora_buffersize, lora_key);
	printf("config lorahat at %d bps, lora airspeed %dbps -> %s\r\n", bps, lora_airspeed, c == 0?"OK":"FAILED");

	if (c == 0) {
		if (default_bps != bps) {
			/* we need to reconnect lorahat's tty */
			close(lora_fd);
			lora_fd = open_tty(lora_tty, bps, 1);
			if (lora_fd < 0) {
				fprintf(stderr, "fail to re-open lorahat serial device %s/%dbps\r\n", lora_tty, bps);
				release_pin(M0);
				release_pin(M1);
				return 1;
			}
			printf("reopen %s - #%d bps -> fd #%d\r\n", lora_tty, bps, lora_fd);
		}

		ev_base = event_base_new();
		if (ev_base != NULL) {
			event_set(&lora_event, lora_fd, EV_READ | EV_PERSIST, handle_lora, NULL);
			event_base_set(ev_base, &lora_event);
			event_add(&lora_event, 0);
			printf("enter event_loop()...\r\n");
			pthread_create(&send_task, NULL, send_handler, NULL);
			pthread_detach(send_task);
			pthread_join(send_task, NULL);
			event_base_dispatch(ev_base);
			/* can't reach here */
		} else {
			fprintf(stderr, "can't allocate event_base\r\n");
		}
	}
	release_pin(M0);
	release_pin(M1);
	release_pin(AUX);
	close(lora_fd);
	return 0;
}
