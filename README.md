# Usage

simple utility to config [waveshare's sx126x lorahat](https://www.waveshare.net/wiki/SX1268_470M_LoRa_HAT).

```
usage: lorahat [optargs]
 -a airspeed, set lora airspeed, default is 38400
 -A lora address, set lora address, default is 0
 -b baudrate, set tty baudrate, default is 115200
 -s tty, set lorahat serial tty, default is /dev/ttyS0
 -f freq, set lora freq in Mhz, default is 433MHz
 -n netid, set lora netid, default is 0
 -p power, set lora tx power in dBm, default is 22dBm
 -z size, set lorahat packet size, default is 240 Bytes, 128/64/32
 -k key, set lora crypt key, default is 0, don't crypt
 -B blocksize, set lora file transfer block size, default is 200
 -v, verbose mode
 -h, print help message
```

# SEND TXT

## sender

```
[lora@rpi8:~/lorahat $] ./lorahat
open /dev/ttyS0 at default 9600 bps -> fd #3
config lorahat at tty 115200bps, airspeed 38400bps, rx 433 MHz, rx addr #0, netid #0, tx pwr 22dBm, packet size #240 -> OK
reopen /dev/ttyS0 - #115200 bps -> fd #3
enter event_loop()...
enter '[s|t]0,433,xxxx' to send message
s0,433,"test888"
send ""test888"" to lora #0 / 433Mhz -> OK
enter '[s|t]0,433,xxxx' to send message
s0,433,test999
send "test999" to lora #0 / 433Mhz -> OK
enter '[s|t]0,433,xxxx' to send message
```

## receiver

```
[lora@lora:~/lorahat $] ./lorahat
open /dev/ttyS0 at default 9600 bps -> fd #3
config lorahat at tty 115200bps, airspeed 38400bps, rx 433 MHz, rx addr #0, netid #0, tx pwr 22dBm, packet size #240 -> OK
reopen /dev/ttyS0 - #115200 bps -> fd #3
enter event_loop()...
enter '[s|t]0,433,xxxx' to send message
rx txt msg: from address 0 / 433MHz, len 9, ""test888""
rx txt msg: from address 0 / 433MHz, len 7, "test999"
```

# SEND FILES

## sender

```
[lora@rpi8:~/lorahat $] ./lorahat
open /dev/ttyS0 at default 9600 bps -> fd #3
config lorahat at tty 115200bps, airspeed 38400bps, rx 433 MHz, rx addr #0, netid #0, tx pwr 22dBm, packet size #240 -> OK
reopen /dev/ttyS0 - #115200 bps -> fd #3
enter event_loop()...
enter '[s|t]0,433,xxxx' to send message
t0,433,tags
send #200 at pos 0 data to lorahat -> OK, 17ms
send #200 at pos 200 data to lorahat -> OK, 18ms
send #200 at pos 400 data to lorahat -> OK, 17ms
send #200 at pos 600 data to lorahat -> OK, 134ms
send #200 at pos 800 data to lorahat -> OK, 17ms
send #200 at pos 1000 data to lorahat -> OK, 18ms
send #200 at pos 1200 data to lorahat -> OK, 134ms
send #200 at pos 1400 data to lorahat -> OK, 17ms
send #200 at pos 1600 data to lorahat -> OK, 135ms
send #200 at pos 1800 data to lorahat -> OK, 17ms
send #200 at pos 2000 data to lorahat -> OK, 134ms
send #200 at pos 2200 data to lorahat -> OK, 17ms
send #200 at pos 2400 data to lorahat -> OK, 17ms
send #200 at pos 2600 data to lorahat -> OK, 135ms
send #200 at pos 2800 data to lorahat -> OK, 17ms
send #200 at pos 3000 data to lorahat -> OK, 18ms
send #200 at pos 3200 data to lorahat -> OK, 18ms
send #200 at pos 3400 data to lorahat -> OK, 134ms
send #200 at pos 3600 data to lorahat -> OK, 17ms
send #200 at pos 3800 data to lorahat -> OK, 134ms
send #200 at pos 4000 data to lorahat -> OK, 17ms
send #200 at pos 4200 data to lorahat -> OK, 135ms
send #200 at pos 4400 data to lorahat -> OK, 17ms
send #200 at pos 4600 data to lorahat -> OK, 134ms
send #200 at pos 4800 data to lorahat -> OK, 17ms
send #200 at pos 5000 data to lorahat -> OK, 135ms
send #125 at pos 5200 data to lorahat -> OK, 11ms
sending 5325/5325 data to lora takes 4 seconds, 10650bps
enter '[s|t]0,433,xxxx' to send message
```

## receiver

```
[lora@lora:~/lorahat $] ./lorahat
open /dev/ttyS0 at default 9600 bps -> fd #3
config lorahat at tty 115200bps, airspeed 38400bps, rx 433 MHz, rx addr #0, netid #0, tx pwr 22dBm, packet size #240 -> OK
reopen /dev/ttyS0 - #115200 bps -> fd #3
enter event_loop()...
enter '[s|t]0,433,xxxx' to send message
write #200 data at pos 0 to file 56469661 -> OK
write #200 data at pos 200 to file 56469661 -> OK
write #200 data at pos 400 to file 56469661 -> OK
write #200 data at pos 600 to file 56469661 -> OK
write #200 data at pos 800 to file 56469661 -> OK
write #200 data at pos 1000 to file 56469661 -> OK
write #200 data at pos 1200 to file 56469661 -> OK
write #200 data at pos 1400 to file 56469661 -> OK
write #200 data at pos 1600 to file 56469661 -> OK
write #200 data at pos 1800 to file 56469661 -> OK
write #200 data at pos 2000 to file 56469661 -> OK
write #200 data at pos 2200 to file 56469661 -> OK
write #200 data at pos 2400 to file 56469661 -> OK
write #200 data at pos 2600 to file 56469661 -> OK
write #200 data at pos 2800 to file 56469661 -> OK
write #200 data at pos 3000 to file 56469661 -> OK
write #200 data at pos 3200 to file 56469661 -> OK
write #200 data at pos 3400 to file 56469661 -> OK
write #200 data at pos 3600 to file 56469661 -> OK
write #200 data at pos 3800 to file 56469661 -> OK
write #200 data at pos 4000 to file 56469661 -> OK
write #200 data at pos 4200 to file 56469661 -> OK
write #200 data at pos 4400 to file 56469661 -> OK
write #200 data at pos 4600 to file 56469661 -> OK
write #200 data at pos 4800 to file 56469661 -> OK
write #200 data at pos 5000 to file 56469661 -> OK
write #125 data at pos 5200 to file 56469661 -> OK
```
