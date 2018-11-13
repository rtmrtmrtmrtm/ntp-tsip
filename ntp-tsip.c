//
// talk TSIP to a Trimble Thunderbolt GPS receiver,
// read PPS pulses, send time to ntpd via "shm" driver.
//
// /etc/ntp.conf should say:
// server 127.127.28.<UNIT> mode 1
//
// then run ntp-tsip as:
// ntp-tsip /dev/cuauX UNIT
//
// includes a hack to fix broken week roll-over in
// older (pre-E) Thunderbolts.
//
// cc -o tsip tsip.c -lm
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/timepps.h>
#include <math.h>
#include <stdarg.h>

#define DLE 0x10
#define ETX 0x03

// use 0x8E-A4 test mode to test leap second.
// #define TEST_MODE

void primary_timing(unsigned char *, int);
void supplemental_timing(unsigned char *, int);
void software_version(unsigned char buf[], int len);
time_t gps_to_unix(int week, int tow);
void send(int, int);
void send5(int id, int a, int b, int c, int d, int e);

int fd; // trimble thunderbolt
int cooking = -1; // 1 = we're reporting times to ntpd
int entered_holdover = 0;

// info from supplemental timeing packet
int discipline_mode = -1;
int printed_pos = 0; // have we printed a position?
int last_survey_progress = -1;
int decoding_status = -1;
int holdover_duration = 0;
int in_holdover = 0;
unsigned short critical_alarms = 0;
unsigned short minor_alarms = 0;

// http://doc.ntp.org/4.2.8/drivers/driver28.html
struct shmTime {
        int    mode; /* 0 - if valid is set:
                      *       use values,
                      *       clear valid
                      * 1 - if valid is set:
                      *       if count before and after read of data is equal:
                      *         use values
                      *       clear valid
                      */
        volatile int    count;
        time_t          clockTimeStampSec;
        int             clockTimeStampUSec;
        time_t          receiveTimeStampSec;
        int             receiveTimeStampUSec;
        int             leap;
        int             precision;
        int             nsamples;
        volatile int    valid;
        unsigned        clockTimeStampNSec;     /* Unsigned ns timestamps */
        unsigned        receiveTimeStampNSec;   /* Unsigned ns timestamps */
        int             dummy[8];
};
struct shmTime *shm;

// print an informative message to stdout,
// with current time.
void
say(char *fmt, ...)
{
  va_list args;

  time_t now;
  time(&now);
  struct tm xxtm;
  gmtime_r(&now, &xxtm);
  printf("%04d-%02d-%02d %02d:%02d:%02d ",
         xxtm.tm_year + 1900,
         xxtm.tm_mon + 1,
         xxtm.tm_mday,
         xxtm.tm_hour,
         xxtm.tm_min,
         xxtm.tm_sec);

  va_start(args, fmt);
  vfprintf(stdout, fmt, args);
  va_end(args);
  printf("\n");
  fflush(stdout);
}

void
init(char *devname, int unit)
{
  struct termios tt;

  setlinebuf(stdout);

  fd = open(devname, O_RDWR);
  if(fd < 0){
    perror(devname);
    exit(1);
  }

  if(tcgetattr(fd, &tt) != 0){
    perror("tcgetattr");
    exit(1);
  }
  cfsetspeed(&tt, B9600);
  cfmakeraw(&tt);
  tt.c_iflag = IGNBRK;
  tt.c_oflag = 0;
  tt.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
  tt.c_lflag = 0;

  if(tcsetattr(fd, TCSANOW, &tt) != 0){
    perror("tcsetattr");
    exit(1);
  }

  int what = FREAD|FWRITE;
  ioctl(fd, TIOCFLUSH, &what);

  send(0x1F, 0x0); // ask for firmware version -> 0x45
  send(0x1F, 0x0); // ask for firmware version -> 0x45
  send5(0x8E, 0xA5, 0, 5, 0, 0); // only timing messages

  // turn off test mode
  {
    unsigned char buf[11];
    int i = 0;
    buf[i++] = DLE;
    buf[i++] = 0x8E;
    buf[i++] = 0xA4;
    buf[i++] = 0; // Test Mode 0
    buf[i++] = DLE;
    buf[i++] = ETX;
    write(fd, buf, i);
  }

#ifdef TEST_MODE
  // ask the thunderbolt to start at 23:59 on Dec 31 2016
  // just before a leap second, to test rb.c.
  // to be used with:
  // killall ntpd ; sleep 1; date -u 201612312359 ; /etc/rc.d/ntpd start
  {
    unsigned int test_week = 905;
    unsigned int test_seconds = (7 * 86400) - 1 - 42;
    unsigned int utc_offset = 17;
    unsigned short leap_week = 906; // week of leap second event
    unsigned short leap_day = 0; // day-of-week of event
    float A0 = 0.0; unsigned char *A0p = (unsigned char *) &A0;
    float A1 = 1.0; unsigned char *A1p = (unsigned char *) &A1;
    {
      unsigned char buf[11];
      int i = 0;
      buf[i++] = DLE;
      buf[i++] = 0x8E;
      buf[i++] = 0xA4;
      buf[i++] = 3; // Test Mode 3 (UTC parameters)
      buf[i++] = A0p[0]; // A_0, offset of GPS from UTC
      buf[i++] = A0p[1];
      buf[i++] = A0p[2];
      buf[i++] = A0p[3];
      buf[i++] = A1p[0]; // A_1, rate of change of offset
      buf[i++] = A1p[1];
      buf[i++] = A1p[2];
      buf[i++] = A1p[3];
      buf[i++] = (utc_offset >> 8) & 0xff; // delta_t_LS, current leap seconds
      buf[i++] = (utc_offset >> 0) & 0xff;
      buf[i++] = 0; // t_ot, reference time-of-week for A0/A1
      buf[i++] = 0;
      buf[i++] = 0;
      buf[i++] = 0;
      buf[i++] = 0; // WN_t reference week number for A0/A1
      buf[i++] = 0;
      buf[i++] = (leap_week >> 8) & 0xff; // WN_LSF, week number of future leap second event
      buf[i++] = (leap_week >> 0) & 0xff; 
      buf[i++] = (leap_day >> 8) & 0xff; // DN, day number of future leap second event
      buf[i++] = (leap_day >> 8) & 0xff;
      buf[i++] = 0; // delta_t_LSF, new leap seconds
      buf[i++] = utc_offset + 1;
      buf[i++] = DLE;
      buf[i++] = ETX;
      write(fd, buf, i);
    }
    {
      unsigned char buf[11];
      int i = 0;
      buf[i++] = DLE;
      buf[i++] = 0x8E;
      buf[i++] = 0xA4;
      buf[i++] = 1; // Test Mode 1
      buf[i++] = (test_week >> 8) & 0xff;
      buf[i++] = (test_week >> 0) & 0xff;
      buf[i++] = (test_seconds >> 24) & 0xff;
      buf[i++] = (test_seconds >> 16) & 0xff;
      buf[i++] = (test_seconds >> 8) & 0xff;
      buf[i++] = (test_seconds >> 0) & 0xff;
      buf[i++] = DLE;
      buf[i++] = ETX;
      write(fd, buf, i);
    }
  }
#endif
  
  // capture PPS from serial port DCD.
  pps_params_t params;
  bzero(&params, sizeof(params));
  params.api_version = PPS_API_VERS_1;
  params.mode = PPS_CAPTUREASSERT;
  if(ioctl(fd, PPS_IOC_SETPARAMS, &params) < 0){
    perror("PPS_IOC_SET_PARAMS");
    exit(1);
  }  

  // talk to the ntpd shared memory driver.
  shm = 0;
  for(int iters = 0; iters < 5; iters++){
    key_t shmkey = 0x4E545030 + unit;
    int shmid = shmget(shmkey, 0, 0);
    if(shmid < 0){
      perror("shmget");
      sleep(2);
      continue;
    }
    void *vshm = shmat(shmid, (void *) 0, 0);
    if(vshm == (void*)-1){
      perror("shmat");
      sleep(2);
      continue;
    }
    shm = (struct shmTime *) vshm;
    break;
  }

  if(shm == 0){
    fprintf(stderr, "could not open shm unit %d\n", unit);
    exit(1);
  }
  
  memset(shm, 0, sizeof(*shm));
  shm->mode = 1; // use count
  shm->precision = -13; // 2^-13 seconds?

  {
    char buf[1024];
    read(fd, buf, sizeof(buf));
  }
}

void
send(int id, int subcode)
{
  unsigned char buf[5];
  int i = 0;
  buf[i++] = DLE;
  buf[i++] = id;
  if(subcode != 0){
    buf[i++] = subcode;
  }
  buf[i++] = DLE;
  buf[i++] = ETX;
  write(fd, buf, i);
}

void
send5(int id, int a, int b, int c, int d, int e)
{
  unsigned char buf[8];
  int i = 0;
  buf[i++] = DLE;
  buf[i++] = id;
  buf[i++] = a;
  buf[i++] = b;
  buf[i++] = c;
  buf[i++] = d;
  buf[i++] = e;
  buf[i++] = DLE;
  buf[i++] = ETX;
  write(fd, buf, i);
}

void
go()
{
  unsigned char buf[4096];
  int n = 0;

  while(1){
    if(n >= sizeof(buf)){
      say("input buffer overflow");
      n = 0;
    }

    assert(n >= 0);
    assert(n < sizeof(buf));

    int cc = read(fd, buf+n, 1);
    if(cc <= 0){
      perror("read");
      exit(1);
    }

    n += 1;

    // is a command present?
    // search for the end of a packet.
    // DLE id ... DLE ETX
    // DLE 0x10
    // ETX 0x03
    // id cannot be 0x10 or 0x03
    // DLE in content is stuffed to be DLE DLE
    // so the end of a packet is <not-DLE> DLE ETX
    int i1 = 0;
    while(i1+2 < n){
      if(buf[i1] != DLE && buf[i1+1] == DLE && buf[i1+2] == ETX){
        // end of packet.
        // if buf[] contains a full packet ending here, process.
        // in any case, toss it.
        // XXX don't toss the final ETX, so next DLE can be shown
        //   to be not escaped DLE DLE.
        int i0;
        // look backwards for DLE that's not DLE DLE.
        for(i0 = i1-1; i0 >= 1; --i0){
          if(buf[i0] == DLE && buf[i0-1] == DLE){
            // skip escaped DLE
            i0 -= 1;
          } else if(buf[i0] == DLE){
            // start!
            break;
          }
        }

        if(i0 > 0 && buf[i0] == DLE && buf[i0-1] != DLE){
          // start of packet.
          // i0 points to starting DLE.
          // i1 points to last byte in payload, just before ending DLE.
          int id = buf[i0+1];
          int subcode = buf[i0+2];
          //int len = i1 - i0 - 2 + 1; // not including ID
          //unsigned char *start = buf + i0 + 2; // byte after ID

          // copy the packet, un-escaping DLEs.
          // we copy starting at the byte after the ID.
          unsigned char payload[1024];
          int len = 0;
          for(int i = i0 + 2; i <= i1; i++){
            assert(len < sizeof(payload));
            payload[len++] = buf[i];
            if(buf[i] == DLE && i+1 <= i1 && buf[i+1] == DLE){
              i += 1;
            }
          }

          if(id == 0x8f && subcode == 0xab){
            primary_timing(payload, len);
          } else if(id == 0x45){
            software_version(payload, len);
          } else if(id == 0x8f && subcode == 0xac){
            supplemental_timing(payload, len);
          } else if(id == 0x8f && subcode == 0xa5){
            // reply to setting packet broadcast mask
          } else {
            say("unknown %02x %02x", id, subcode);
          }
        } else {
          say("pkt w/o start");
        }

        // toss the [broken] packet up through ending DLE ETX.
        memmove(buf, buf + i1 + 1, n - (i1 + 1));
        n -= i1 + 1;
        i1 -= (i1 + 1);
      }

      i1 += 1;
    }
  }
}

#define NMEDIAN 5
struct {
  time_t clock;
  double pps;
} median[NMEDIAN];
int median_n = 0;
int median_i = 0;

// median of recent time offsets.
// a PPS interrupt occured at UNIX time pps,
// which the external clock claimed
// was at time time.
void
median_add(time_t clock, double pps)
{
  median[median_i].clock = clock;
  median[median_i].pps = pps;
  median_i = (median_i + 1) % NMEDIAN;
  if(median_n < NMEDIAN)
    median_n += 1;
}

// caller indicates desired external clock time.
// returns plausible pps in *pps.
// discards min/max outliers, then averages.
void
median_get(time_t clock, double *pps)
{
  assert(median_n > 0);
  if(median_n < NMEDIAN){
    int i = median_n - 1;
    long long diff = clock - median[i].clock;
    *pps = median[i].pps + diff;
    return;
  }

  int mini = -1;
  double min = 0;
  int maxi = -1;
  double max = 0;
  for(int i = 0; i < NMEDIAN; i++){
    double d = median[i].pps - median[i].clock;
    if(mini == -1 || d < min){
      mini = i;
      min = d;
    }
    if(maxi == -1 || d > max){
      maxi = i;
      max = d;
    }
  }

  double sum = 0.0;
  int n = 0;
  for(int i = 0; i < NMEDIAN; i++){
    if(i != mini && i != maxi){
      long long diff = clock - median[i].clock;
      sum += median[i].pps + diff;
      n += 1;
    }
  }

  *pps = sum / n;
}

// 8f ab
// begins 20ms after the PPS pulse.
void
primary_timing(unsigned char buf[], int len)
{
  if(len != 17){
    say("8f ab len=%d wanted 17", len);
    return;
  }

  // lame guess about local clock time at which
  // we heard the tick from the Thunderbolt.
  struct timeval recv;
  gettimeofday(&recv, (struct timezone *) 0);

  // the thunderbolt starts this packet 20 ms after the
  // seconds, and it's about 22 bytes long at 9600 baud.
  int adj = 42 * 1000; // usec
  if(recv.tv_usec < adj){
    recv.tv_sec -= 1;
    recv.tv_usec = 1000000 - adj + recv.tv_usec;
  } else {
    recv.tv_usec -= adj;
  }

  int utc_offset = (buf[7] << 8) | buf[8];

  int is_utc = buf[9] & (1 << 0); // 0=GPS time, 1=UTC time
  int time_not_set = buf[9] & (1 << 2); // 0=time is set, 1=not set
  int no_utc = buf[9] & (1 << 3); // 0=have GPS-UTC seconds, 1=not
  int test_mode = buf[9] & (1 << 4); // 0=time from GPS, 1=not

  if(time_not_set){
    if(cooking && in_holdover && holdover_duration < 600){
      if(entered_holdover == 0){
        say("entering holdover");
        entered_holdover = 1;
      }
    } else {
      if(cooking){
        say("thunderbolt says time is not set; in_holdover %d duration %d",
          in_holdover, holdover_duration);
        entered_holdover = 0;
      }
      cooking = 0;
      return;
    }
  } else {
    if(entered_holdover){
      say("exiting holdover");
    }
    entered_holdover = 0;
  }

  if(no_utc){
    if(cooking){
      say("thunderbolt says no UTC info");
    }
    cooking = 0;
    return;
  }

#ifndef TEST_MODE
  if(test_mode){
    if(cooking){
      say("thunderbolt says time from user");
    }
    cooking = 0;
    return;
  }
#endif

  int week = (buf[5] << 8) | buf[6];
  int tow =
    (buf[1] << 24) |
    (buf[2] << 16) |
    (buf[3] << 8) |
    buf[4];

  //printf("week %d tow %d\n", week, tow);

  // translate GPS time to UNIX seconds-since-1970,
  // though without UTC seconds correction.
  time_t ugps = gps_to_unix(week, tow);

  ugps -= utc_offset;

  // get local UNIX time of PPS from serial line DCD interrupt.
  struct pps_fetch_args fetch;
  bzero(&fetch, sizeof(fetch));
  fetch.tsformat = PPS_TSFMT_TSPEC;
  if(ioctl(fd, PPS_IOC_FETCH, &fetch) < 0){
    perror("PPS_IOC_FETCH");
    exit(1);
  }
  struct timespec pps;
  pps = fetch.pps_info_buf.assert_tu.tspec;

  // did PPS interrupt close to the time when we
  // received the "primary" packet from the Thunderbolt?
  // this yields a false positive once during a leap second.
  double t1 = recv.tv_sec + recv.tv_usec / 1000000.0;
  double t2 = pps.tv_sec + pps.tv_nsec / 1000000000.0;
  if(fabs(t1 - t2) >= 0.2){
    if(cooking){
      say("crazy PPS %f, recv %f, pps %f", fabs(t1 - t2), t1, t2);
    }
    cooking = 0;
    return;
  }

  if(cooking != 1){
    struct tm xxtm;
    gmtime_r(&ugps, &xxtm);
    char tbuf[128];
    sprintf(tbuf, "%04d-%02d-%02d %02d:%02d:%02d %f",
            xxtm.tm_year + 1900,
            xxtm.tm_mon + 1,
            xxtm.tm_mday,
            xxtm.tm_hour,
            xxtm.tm_min,
            xxtm.tm_sec,
            pps.tv_nsec / 1000000000.0);
    say("cooking, utc_offset %d, %s", utc_offset, tbuf);
    cooking = 1;
  }

#if 0
  struct tm xxtm;
  gmtime_r(&ugps, &xxtm);
  printf("thunderbolt: %04d %02d %02d %02d:%02d:%02d %f\n",
         xxtm.tm_year + 1900,
         xxtm.tm_mon,
         xxtm.tm_mday,
         xxtm.tm_hour,
         xxtm.tm_min,
         xxtm.tm_sec,
	 pps.tv_nsec / 1000000000.0);

  gmtime_r(&recv.tv_sec, &xxtm);
  printf("real:  %04d %02d %02d %02d:%02d:%02d %f\n",
         xxtm.tm_year + 1900,
         xxtm.tm_mon,
         xxtm.tm_mday,
         xxtm.tm_hour,
         xxtm.tm_min,
         xxtm.tm_sec,
         recv.tv_usec / 1000000.0);


#if 0
  printf("thunderbolt: %04d %02d %02d %02d:%02d:%02d\n",
         (buf[15] << 8) | buf[16], // year
         buf[14], // month, 1..12
         buf[13], // day of month, 1..31
         buf[12], // hours, 0..23
         buf[11], // minutes
         buf[10]); // seconds
#endif
#endif

  int leap = 0;
  if(minor_alarms & 0x80){
    // thunderbolt says a leap second is drawing near,
    // but doesn't say which way, or when.
    struct tm xxtm;
    gmtime_r(&ugps, &xxtm);
    if(xxtm.tm_mon+1 == 6 || xxtm.tm_mon+1 == 12){
      // only allowed in June or December.
      leap = 1; // assume LEAP_ADDSECOND
    }
  }

  double dpps = pps.tv_sec + (pps.tv_nsec / 1000000000.0);

  median_add(ugps, dpps);
  double mpps;
  median_get(ugps, &mpps);

#if 0
  printf("%f %f\n", (dpps - ugps) * 1000.0, (mpps - ugps) * 1000.0);
#endif

  shm->valid = 0;
  __sync_synchronize();
  shm->clockTimeStampSec = ugps;
  shm->receiveTimeStampSec = trunc(mpps);
  shm->receiveTimeStampUSec = (mpps - trunc(mpps)) * 1000000;
  shm->leap = leap;
  shm->count += 1;
  __sync_synchronize();
  shm->valid = 1;
}

// turn week and time-of-week (in seconds) into UNIX time.
// GPS week zero started at 00:00:00 UTC on January 6, 1980.
// first roll-over 23:59:47 UTC on August 21, 1999.
// probably does the wrong thing during a leap second.
// caller should add GPS/UTC offset to the return value.
time_t
gps_to_unix(int week, int tow)
{
  // turn GPS epoch into a UNIX time.
  struct tm eptm;
  memset(&eptm, 0, sizeof(eptm));
  eptm.tm_sec = 0;
  eptm.tm_min = 0;
  eptm.tm_hour = 0;
  eptm.tm_mday = 6;
  eptm.tm_mon = 0;
  eptm.tm_year = 1980 - 1900;
  time_t ept = timegm(&eptm);

  if(week < 1024){
    // old Thunderbolt with stale roll-over.
    // (Thunderbolt-E reports 16 bit weeks!)

    // advance to post-1999 GPS epoch.
    ept += 1024 * 7 * 86400;
    
    if(week < 990){
      // advance to post-2019 GPS epoch.
      ept += 1024 * 7 * 86400;
    }
  }

  // a unix time is really days since 1970 * 86400 plus
  // seconds into the day. it is not really seconds since 1970.
  ept += week * 7 * 86400;
  ept += tow;

  return ept;
}

// copy 8 bytes, reversing order.
void
rev8(void *xto, void *xfrom)
{
  char *to = (char *) xto;
  char *from = (char *) xfrom;
  for(int i = 0; i < 8; i++){
    to[i] = from[7-i];
  }
}

void
dms(double x, char *buf)
{
  int d = x;
  x -= (int) x;
  x *= 60;
  if(x < 0){
    x = 0.0 - x;
  }
  int m = x;
  x -= (int) x;
  x *= 60;
  double s = x;
  sprintf(buf, "%d %d %.3f", d, m, s);
}

// 8f ac
void
supplemental_timing(unsigned char buf[], int len)
{
  if(len != 68){
    say("8f ac len=%d wanted 68", len);
    return;
  }

  char *snames[] = { "doing fixes", "don't have GPS time",
                     "2", "PDOP is too high", "4", "5", "6", "7",
                     "no usable sats",
                     "only 1 usable sat",
                     "only 2 usable sat",
                     "only 3 usable sats",
                     "the chosen sat is unusable", "13", "14", "15",
                     "TRAIM rejected the fix", "17" };
  if(decoding_status != buf[12]){
    char *old = "";
    if(decoding_status != -1){
      old = snames[decoding_status];
    }
    say("decoding status %s -> %s", old, snames[buf[12]]);
    decoding_status = buf[12];
  }
                    
  char *dnames[] = { "normal", "power-up", "auto holdover",
                     "manual holdover", "recovery", "not used",
                     "disabled", "???" };
  if(discipline_mode != buf[2]){
    char *old = "";
    if(discipline_mode != -1){
      old = dnames[discipline_mode];
    }
    say("discipline %s -> %s", old, dnames[buf[2]]);
    discipline_mode = buf[2];
  }

  if(buf[2] == 2){
    // auto holdover
    in_holdover = 1;
  } else {
    in_holdover = 0;
  }

  holdover_duration =
    (buf[4] << 24) |
    (buf[5] << 16) |
    (buf[6] << 8) |
    buf[7];

  // did the survey just now complete?
  int survey_done = 
     (last_survey_progress > 0 && last_survey_progress < 100 &&
      buf[3] == 100);

  // 36-43 latitude, double, radians
  // 44-51 longitude, double, radians
  // 52-59 altitude, double, meters
  // ansi standard double, most-significant byte first
    
  double lat, lon, alt;
    
  rev8(&lat, buf+36);
  rev8(&lon, buf+44);
  rev8(&alt, buf+52);
  
  lat = 360.0 * lat / (2 * M_PI);
  lon = 360.0 * lon / (2 * M_PI);
    
  if((buf[3] > 1) && (printed_pos == 0 || survey_done)){
    char latbuf[64];
    dms(lat, latbuf);
    char lonbuf[64];
    dms(lon, lonbuf);

    say("survey %d%%: %s, %s, %.1f", buf[3], latbuf, lonbuf, alt);

    printed_pos = 1;
  }

  last_survey_progress = buf[3]; // 0..100%

  // critical alarms
  unsigned short critical = (buf[8] << 8) | buf[9];
  if(critical != critical_alarms){
    say("critical alarms %04x", critical);
    critical_alarms = critical;
  }

  // minor alarms
  unsigned short minor = (buf[10] << 8) | buf[11];
  if(minor != minor_alarms){
    say("minor alarms %04x", minor);
    minor_alarms = minor;
  }

  // leap second warning?
  if(minor & 0x80){
    // thunderbolt does not say if it will be added or
    // subtracted, and does not say when.
  }
}

// 45
void
software_version(unsigned char buf[], int len)
{
  if(len != 10){
    say("45 len=%d wanted 10", len);
    return;
  }
  say("version %d.%d %d %d %d, %d.%d %d %d %d",
         buf[0], // application major
         buf[1], // application minor
         buf[2], // month
         buf[3], // day
         buf[4] + 2000,
         buf[5], // gps core major
         buf[6], // gps core minor
         buf[7], // month
         buf[8], // day
         buf[9] + 2000);
}

int
main(int argc, char *argv[])
{
  if(argc != 3){
    fprintf(stderr, "Usage: tsip serial-device shm-unit\n");
    exit(1);
  }
  init(argv[1], atoi(argv[2]));
  go();
}
