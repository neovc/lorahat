#include <stdio.h>
#include <getopt.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

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

	return 0;
}
