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

int bps = 9600, lora_airspeed = 9600, lora_freq = 433, lora_netid = 0, lora_power = 22, lora_buffersize = 1000, verbose_mode = 0;
uint16_t lora_key = 0;
char lora_tty[50] = "/dev/ttyS0";
int lora_fd = -1;

#define M0 "22"
#define M1 "27"
#define AUX "4"

void
print_help(void)
{
	printf("usage: loraip [optargs]\r\n"
           " -a airspeed, set lora airspeed, default is 9600\r\n"
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
open_tty(char *dev, int speed)
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

	/* set non-blocking tty fd */
	flag = fcntl(fd, F_GETFL, 0);
	if (flag >= 0)
		fcntl(fd, F_SETFL, flag | O_NONBLOCK);

	/* Flush old transmissions */
	if (tcflush(fd, TCIOFLUSH) == -1)
		fprintf(stderr, "flushing serial port %s failed, %s\r\n", dev, strerror(errno));

	return fd;
}

int
main(int argc, char **argv)
{
	int c;

	while ((c = getopt(argc, argv, "a:b:z:s:p:n:k:f:v")) != -1) {
		switch (c) {
			case 'v':
				verbose_mode = 1;
				break;
			case 'a':
				lora_airspeed = atoi(optarg);
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
	lora_fd = open_tty(lora_tty, bps);
	if (lora_fd < 0) {
		fprintf(stderr, "fail to open lorahat serial device %s\n", lora_tty);
		return 1;
	}

	if (setup_pin(M0, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\n", M0, OUTPUT);
		close(lora_fd);
		return 1;
	}

	if (setup_pin(M1, OUTPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\n", M1, OUTPUT);
		release_pin(M0);
		close(lora_fd);
		return 1;
	}

	if (setup_pin(AUX, INPUT)) {
		fprintf(stderr, "failed to set pin #%s-%s failed\n", AUX, INPUT);
		release_pin(M0);
		release_pin(M1);
		close(lora_fd);
		return 1;
	}

	release_pin(M0);
	release_pin(M1);
	release_pin(AUX);
	close(lora_fd);
	return 0;
}
