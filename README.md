# ntp-tsip

talk TSIP to a Trimble Thunderbolt GPS receiver,
read PPS pulses, send time to ntpd via "shm" driver.

/etc/ntp.conf should say:
server 127.127.28.<UNIT> mode 1

then run ntp-tsip as:
ntp-tsip /dev/cuauX UNIT

includes a hack to fix broken week roll-over in
older (pre-E) Thunderbolts.

cc -o tsip tsip.c -lm
