/*
  Ausleseprogram für Voltcraft VC820 & VC840

  Reverse-Engineered von Georg Acher, Nov. 2001

  WWW:    http://www.acher.org
  e-mail: georg at acher dot org

  Das VC820/840 sendet 14 Bytes mit 2400Baud 8N1
  Das High-Nibble der Bytes entspricht der Position (1...e)
  Nur das Low-Nibble ist interessant!
  Die Low-Nibbles codieren LCD-Segmente:
  Nibble 0: AUTO|AC|TRUE_RMS|RS232
  Nibble 1-2: Minus, Digit 0
  Nibble 3-4: Dot, Digit 1
  Nibble 5-6: Dot, Digit 2
  Nibble 7-8: Dot, Digit 3
  Nibble 8-14: numkM A V Hz F Ohm °C % Rel Diode Beep Hold

------------------------------------------------------------------------------------------------------

Eingebaut von Rolf Fretag, 2004:
- Timeouts mittels pthead, select
- Konsistenzprüfung mittels Empfangzeit-Messung
- 10 ms check if nothing is received before the first byte of a packet is received.
- 10 ms check if nothing is received after the last byte of a packet is received.
- no wrap around of get_time

Im Gegensatz zu früher werden die empfangenen Datenpakete dadurch fast immer richtig erkannt
und defekte Datenpakete werden nun korrekt verworfen.
Dies ist wichtig wenn die Übertragung durch ein sehr schwaches Signal oder einen Wackelkontakt
stark gestört ist.


TODO:
- Herausfinden, warum (unter Linux Kernel 2.6.8) ein Killen der Original-Version des Programms mit Ctrl-C
  zur ca. 5 % dazu führt, dass das Device danach (bis zum reboot) nicht mehr geöffnet werden kann, obwohl
  kein Prozess mehr darauf zugreift!


Es können auch mehrere dieser Programme eine Schnittstelle gleichzeitig auslesen, aber da die Daten nicht
dupliziert werden, also einzelne Bytes immer von nur einem Prozess gelesen werden, gibt das etwas Datenmüll
und einige timeouts, funktioniert aber.


Licence: GPL (GNU PUBLIC LICENCE).

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <iso646.h>
#include <string.h>


#define STD_FLG (CS8|CREAD|HUPCL)       // standard flags (8N1)

// Dividing of (unsigned) integer numbers with good rounding (with minimized quantisation error).
// Integer division (of positive numbers) ignores everything behind the point while good rounding
// rounds up >= .5 and rounds down < .5 (kaufmännische Rundung).
#define mc_POS_DIV(a, b)  ( (a)/(b) +  ( ( (a) % (b) >= (b)/2 ) ? 1 : 0 ) )


int fd;                         // file descriptor


// time with microseconds
inline signed long long int
get_time ()
{
  static struct timeval ti;
  static struct timezone tzp;

  gettimeofday (&ti, &tzp);
  return (ti.tv_usec + 1000000 * ((long long int) time (NULL)));
}


/*---------------------------------------------------------------------*/
int
open_serial (const char *device, int baudrate)
{
  int fd = 0;
  int bdflag = 0;
  static struct termios tty;
  int modelines = 0;

  fprintf (stderr, "Using serial output to %s, baudrate %i\n", device, baudrate);
  switch (baudrate)
  {
    case 1200:
      bdflag = B1200;
      break;
    case 2400:
      bdflag = B2400;
      break;
    case 4800:
      bdflag = B4800;
      break;
    case 9600:
      bdflag = B9600;
      break;
    case 19200:
      bdflag = B19200;
      break;
    case 38400:
      bdflag = B38400;
      break;
    case 57600:
      bdflag = B57600;
      break;
    case 115200:
      bdflag = B115200;
      break;
    case 230400:
      bdflag = B230400;
      break;
    default:
      fprintf (stderr, "Unsupported baudrate %i\n", baudrate);
      exit (-1);
  }
  /*Öffnen und Überprüfen des Filedeskriptors with the read+write mode and waitig mode */
  fd = open (device, O_RDWR);
  if (fd == -1)
  {
    perror ("open failed");
    exit (-1);
  }
  if (!isatty (fd))
  {
    fprintf (stderr, "The Device %s is no Terminal-Device !\n", device);
    return (-1);
  }
  fprintf (stderr, "fopen %s done.\n", device);
  tcgetattr (fd, &tty);
  tty.c_iflag = IGNBRK | IGNPAR;        /*Ignoriere BREAKS beim Input *//*Keine Parität */
  tty.c_oflag = IGNPAR;         /*Keine Parität */
  tty.c_lflag = 0;              /*Keine besonderen Angaben nötig */
  tty.c_line = 0;
  tty.c_cc[VTIME] = 0;          // no waiting
  tty.c_cc[VMIN] = 1;           // minimum of reading bytes
  tty.c_cflag = CS8 | CREAD | CLOCAL | HUPCL;   // was STD_FLG; 8N1, read, no modem status,
  cfsetispeed (&tty, bdflag);   // input
  cfsetospeed (&tty, bdflag);   // output
  fprintf (stderr, "cfsetispeed and cfsetospeed done.\n");
  if (-1 == tcsetattr (fd, TCSAFLUSH, &tty))
  {
    fprintf (stderr, "Error: Could not set terminal attributes for %s !\n", device);
  }
  /* Lesen der IO-Control-Parameter zum Testen */
  if (ioctl (fd, TIOCMGET, &modelines) == -1)
  {
    fprintf (stderr, "Fehler bei ioctl TIOCMGET auf  %s!\n", device);
    return (-1);
  }
  fprintf (stderr, "ioctl done.\n");
  return (fd);
}


/*---------------------------------------------------------------------*/
void
close_serial (int fd)
{
  close (fd);
  return;
}


/*---------------------------------------------------------------------*/
void
set_rts_dtr (int fd)
{
  int arg = 0;

  arg = TIOCM_RTS | TIOCM_DTR;
  ioctl (fd, TIOCMBIS, &arg);
  arg = TIOCM_RTS;
  ioctl (fd, TIOCMBIC, &arg);
  return;
}


/*---------------------------------------------------------------------*/
int
digit (int x)
{
  int dig[10] = { 0xd7, 0x05, 0x5b, 0x1f, 0x27, 0x3e, 0x7e, 0x15, 0x7f, 0x3f };
  int n = 0;
  x = x & 0x7f;
  for (n = 0; n < 10; n++)
  {
    if (x == dig[n])
      return n;
  }
  return 0;
}


/*---------------------------------------------------------------------*/
void
dvm_unit (int y, int x, char *s)
{
  char *prefix = "";
  char *unit = "";
  char *ext = "";
  //char *ext1 = "";

  if (x & 0x2000)
    ext = "delta";
  else
  {
    if (x & 0x100000)
      ext = "Diode";
    else
    {
      if (x & 0x10000)
        ext = "Beep";
    }
  }
  // prefix: m or u or n ...
  if (x & 0x080000)
    prefix = "m";
  else
  {
    if (x & 0x800000)
      prefix = "u";
    else
    {
      if (x & 0x400000)
        prefix = "n";
      else
      {
        if (x & 0x020000)
          prefix = "M";
        else
        {
          if (x & 0x200000)
            prefix = "k";
        }
      }
    }
  }
  // unit: A or V or Hz or ...
  if (x & 0x0800)
    unit = "A";
  else
  {
    if (x & 0x0200)
      unit = "Hz";
    else
    {
      if (x & 0x40000)
        unit = "%";
      else
      {
        if (x & 0x10)
          unit = "°C";
        else
        {
          if (x & 0x4000)
            unit = "Ohm";
          else
          {
            if (x & 0x0400)
              unit = "V";
            else
            {
              if (x & 0x8000)
                unit = "F";
            }
          }
        }
      }
    }
  }
  snprintf (s, 128, "%s%s %s (%s)", prefix, unit, (y & 0x8 ? "AC" : ""), ext);
  return;
}


// simple timeout
void *
func_timeout (void *threadid)
{
  sleep (5);                    // 5 s timeout
  if (0 >= fd)                  // device file could not be opened
  {
    fprintf (stderr, "Error: Timeout, device file could not be opened, exiting.\n");
    exit (-1);
  }
  pthread_exit (NULL);
}

double getSeconds(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((double)tv.tv_sec + (double)tv.tv_usec * 1.0e-6);
}


/*---------------------------------------------------------------------*/
int
main (int argc, char **argv)
{
  const char *device = "/dev/ttyS0";    // hard coded first serial port
  unsigned char buffer[100] = { 0 };
  unsigned char buffer1[9] = { 0 };
  char units[20] = { 0 };
  signed int n = 0;
  float it = 0.;
  signed long long int t = 0, tf = 0;
  signed int maxfd = 0, id = 0, i_ret = 0;
  static struct timeval timeout;
  static fd_set readfs;         // file descriptor set
  static pthread_t thread;
  signed long long int lli_rec_time = 0, timeval0 = 0, timeval1 = 0;

  signed long long lastLoggedTime, currentTime, logInterval;

  if (argc > 1) {
	  logInterval = (signed long long )(atof(argv[1]) * 1000000.0);
  } else {
	  logInterval = 1000000;
  }
printf("logInterval %d\n",logInterval); 
  // create thread for timeout (to avoid hangup in open_serial, e. g. when the device can not be opened)
  i_ret = pthread_create (&thread, NULL, func_timeout, (void *) id);    //(void *(*)(void *))
  if (i_ret)
  {
    printf ("ERROR; return code from pthread_create() is %d\n", i_ret);
    exit (-1);
  }
  fd = open_serial (device, 2400);
  maxfd = fd + 1;               // maximum bit entry
  if (fd <= 0)
  {
    fprintf (stderr, "Error: Could not open port, exiting.\n");
    exit (-1);
  }
  set_rts_dtr (fd);             // DTR/RTS setzen
  t = get_time (NULL);
  for (;;)
  {
    // check the 10 ms before a data packet
    FD_ZERO (&readfs);
    FD_SET (fd, &readfs);
    memset (&timeout, 0, sizeof (timeout));
    timeout.tv_usec = 10000;    // 10 ms
    if (select (maxfd, &readfs, NULL, NULL, &timeout))  // no timeout
    {
      fprintf (stderr, "Received data packet fragment (early).\n");
      read (fd, &buffer[n], 1); // empty buffer
      // usleep (50000);           // wait 50 ms to get into a time between the actual and next data packet
      continue;                 // restart
    }
    
	n = 0;
    memset (buffer, 0, sizeof (buffer));
    for (;;)                    // receive the bytes one by one
    {
      memset (&timeout, 0, sizeof (timeout));
      timeout.tv_sec = 5;       // seconds (will be set to zero after select; don't move)
      FD_ZERO (&readfs);
      FD_SET (fd, &readfs);
      if (0 != select (maxfd, &readfs, NULL, NULL, &timeout))
      {
        read (fd, &buffer[n], 1);
        if ((buffer[n] & 0xf0) == 0xe0 || (16 <= n))
          break;
        n++;
        if (1 == n)             // start time counter for reading data
          timeval0 = get_time (NULL);
      }
      else
      {
        fprintf (stderr, "Connection timeout (select failed).\n");
      }
    }
    if (0 == n)                 // nothing received
      continue;
    timeval1 = get_time (NULL);
    lli_rec_time = timeval1 - timeval0;
    if ((lli_rec_time > 100000) or (lli_rec_time < 25000))      // more than 100 ms or less than 25 ms: invalid data
    {
      //fprintf (stderr, "Receive timeout, invalid data packet (took %lld microseconds and must be between 25000 and 100000).\n", lli_rec_time);
      continue;
    }
    // check the 10 ms after a data packet
    FD_ZERO (&readfs);
    FD_SET (fd, &readfs);
    memset (&timeout, 0, sizeof (timeout));
    timeout.tv_usec = 10000;    // 10 ms
    if (select (maxfd, &readfs, NULL, NULL, &timeout))
    {
      fprintf (stderr, "Received data packet fragment (late).\n");
      read (fd, &buffer[n], 1); // empty buffer
      // usleep (50000);           // wait 50 ms to get into a time between the actual and next data packet
      continue;
    }
#ifdef DEBUG
    // Raw output
    for (n = 0; n < 16; n++)
      printf ("%02x ", buffer[n]);
#endif
    buffer1[0] = buffer[0] & 15;
    for (n = 0; n < 8; n++)
      buffer1[1 + n] = ((buffer[2 * n + 1] & 15) << 4) | (buffer[2 * n + 2] & 15);
#ifdef DEBUG
    // Nibble compacted data
    for (n = 0; n < 8; n++)
      printf ("%02x ", buffer1[n]);
    printf ("%i%i%i%i\n", digit (buffer1[1]), digit (buffer1[2]), digit (buffer1[3]), digit (buffer1[4]));
#endif
    if ((buffer1[3] & 0x7f) == 0x68)
      it = 9999999;
    else
      it = 1000.0 * digit (buffer1[1]) + 100.0 * digit (buffer1[2])     //
        + 10 * digit (buffer1[3]) + 1 * digit (buffer1[4]);
    // Dezimalpunkt
    if (buffer1[4] & 0x80)
      it = it / 10.0;
    if (buffer1[3] & 0x80)
      it = it / 100.0;
    if (buffer1[2] & 0x80)
      it = it / 1000.0;
    if (buffer1[1] & 0x80)
      it = -it;
    tf = mc_POS_DIV ((get_time (NULL) - t), 100000);    // tf in 100 ms
    dvm_unit (buffer1[0], (buffer1[5] << 16) | (buffer1[6] << 8) | buffer1[7], units);
    
    currentTime = get_time();
    
    if ((currentTime - lastLoggedTime) >= logInterval) {
      lastLoggedTime = currentTime;
      printf ("%0.1f %0.3f %s\n", tf / 10., it, units);
      fflush (stdout);
    }
    // usleep(100); // wait 100 ms for next reading/writing (not really nessisary with VC820/VC840)
  }
  pthread_exit (NULL);
}
