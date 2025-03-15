/*
 * rs232.c
 *
 *  Created on: 2019年6月11日
 *      Author: a
 */


#include "rs232.h"
int Cport,error;
struct termios new_port_settings,
old_port_settings;
#define OHFLOW      0x00000002
#define IHFLOW      0x00000001

int OpenComport(const char *comport, int baudrate)
{
  int baudr;
  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  Cport = open(comport, O_RDWR | O_NOCTTY | O_NDELAY);
  if(Cport==-1)
  {
    perror("unable to open comport ");
    return(1);
  }

  error = tcgetattr(Cport, &old_port_settings);
  if(error==-1)
  {
    close(Cport);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
  new_port_settings.c_iflag = IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */
  error = tcsetattr(Cport, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(Cport);
    perror("unable to adjust portsettings ");
    return(1);
  }

  return(0);
}
/*
int OpenComport(char *comport, int baudrate,int databits,int parity,int stopbits,int flowcontrol)
{
	Cport = open(comport, O_RDWR | O_NOCTTY | O_NONBLOCK );
	if(Cport==-1)
	{
		perror("unable to open comport ");
		return(1);
	}

	struct termios termios_new;
	bzero(&termios_new, sizeof(termios_new));

	//set c_cflag of the termios, why ?
	termios_new.c_cflag |= CLOCAL | CREAD;//define local and read properities;
	termios_new.c_cflag &= ~CSIZE;//shut off the mask bit;
	termios_new.c_cflag |= BRKINT;//signal interrupt on break;

	//set baudrate
	int set_ispeed_ret = -1;//the return value of setting ispeed;
	int set_ospeed_ret = -1;//the return value of setting ospeed;
	switch(baudrate)
	{
	case 600:
		set_ispeed_ret = cfsetispeed(&termios_new, B600);
		set_ospeed_ret = cfsetospeed(&termios_new, B600);
		break;

	case 1200:
		set_ispeed_ret = cfsetispeed(&termios_new, B1200);
		set_ospeed_ret = cfsetospeed(&termios_new, B1200);
		break;

	case 2400:
		set_ispeed_ret = cfsetispeed(&termios_new, B2400);
		set_ospeed_ret = cfsetospeed(&termios_new, B2400);
		break;

	case 4800:
		set_ispeed_ret = cfsetispeed(&termios_new, B4800);
		set_ospeed_ret = cfsetospeed(&termios_new, B4800);
		break;

	case 9600:
		set_ispeed_ret = cfsetispeed(&termios_new, B9600);
		set_ospeed_ret = cfsetospeed(&termios_new, B9600);
		break;

	case 19200:
		set_ispeed_ret = cfsetispeed(&termios_new, B19200);
		set_ospeed_ret = cfsetospeed(&termios_new, B19200);
		break;

	case 38400:
		set_ispeed_ret = cfsetispeed(&termios_new, B38400);
		set_ospeed_ret = cfsetospeed(&termios_new, B38400);
		break;

	case 57600:
		set_ispeed_ret = cfsetispeed(&termios_new, B57600);
		set_ospeed_ret = cfsetospeed(&termios_new, B57600);
		break;

	case 115200:
		set_ispeed_ret = cfsetispeed(&termios_new, B115200);
		set_ospeed_ret = cfsetospeed(&termios_new, B115200);
		break;

	default:
		fprintf(stderr, "Error: invalid baudrate, at file: %s, function: %s, line: %d;\n", __FILE__, __FUNCTION__, __LINE__);
		return (-1);
	}

	if(-1==set_ispeed_ret)
	{
		fprintf(stderr, "Error: set input baudrate error, at file: %s, function: %s, line: %d, system error info: %s;\n", __FILE__, __FUNCTION__, __LINE__, strerror(errno));
		return (-1);
	}

	if(-1==set_ospeed_ret)
	{
		fprintf(stderr, "Error: set output baudrate error, at file: %s, function: %s, line: %d, system error info: %s;\n", __FILE__, __FUNCTION__, __LINE__, strerror(errno));
		return (-1);
	}

	//set databits
	switch(databits)
	{
	case 5:
		termios_new.c_cflag |= CS5;//5 databits;
		break;
	case 6:
		termios_new.c_cflag |= CS6;//5 databits;
		break;
	case 7:
		termios_new.c_cflag |= CS7;//7 databits;
		break;
	case 8:
		termios_new.c_cflag |= CS8;//8 databits;
		break;
	default:
		fprintf(stderr, "Error: invalid databits, at file: %s, function: %s, line: %d;\n", __FILE__, __FUNCTION__, __LINE__);
		return (-1);
	}

	//set stopbits
	switch (stopbits)
	{
	case 1:
		termios_new.c_cflag &= ~CSTOPB;
		break;
	case 2:
		termios_new.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr, "Error: invalid stopbits, at file: %s, function: %s, line: %d;\n", __FILE__, __FUNCTION__, __LINE__);
		return (-1);
	}

	//set parity
	switch (parity)
	{
	case 1:
		termios_new.c_cflag &= ~PARENB;//no parity
		break;
	case 2:
		termios_new.c_cflag |= PARENB;
		termios_new.c_cflag |= PARODD;//odd parity
		break;
	case 3:
		termios_new.c_cflag |= PARENB;
		termios_new.c_cflag &= ~PARODD;//even parity
		break;
	default:
		fprintf(stderr, "Error: invalid parity, at file: %s, function: %s, line: %d;\n", __FILE__, __FUNCTION__, __LINE__);
		return (-1);
	}


	//set flow control
	switch(flowcontrol)
	{
	case 1:
		termios_new.c_cflag &= ~IXOFF;
		termios_new.c_cflag &= ~IXON;
		termios_new.c_cflag &= ~IHFLOW;
		termios_new.c_cflag &= ~OHFLOW;
		break;
	case 2:
		termios_new.c_cflag &= ~IXOFF;
		termios_new.c_cflag &= ~IXON;
		termios_new.c_cflag |= IHFLOW;
		termios_new.c_cflag |= OHFLOW;
		break;
	case 3:
		termios_new.c_cflag |= IXOFF;
		termios_new.c_cflag |= IXON;
		termios_new.c_cflag &= ~IHFLOW;
		termios_new.c_cflag &= ~OHFLOW;
		break;
	default:
		fprintf(stderr, "Error: invalid flow control, at file: %s, function: %s, line: %d;\n", __FILE__, __FUNCTION__, __LINE__);
		return (-1);
	}

	termios_new.c_cc[VTIME] = 0;//set time which is hanging on for data as 0;
	termios_new.c_cc[VMIN] = 0;//set the minimum number of data received as 0;

	tcflush(Cport, TCIFLUSH);//refresh the data received but not read it into memory

	termios_new.c_cflag |= BRKINT;

	if(-1==tcsetattr(Cport, TCSANOW, &termios_new))
	{
		fprintf(stderr, "Error: set serial port attribute error, at file: %s, function: %s, line: %d, system error info: %s;\n", __FILE__, __FUNCTION__, __LINE__, strerror(errno));
		return (-1);
	}

	return Cport;
}*/


int PollComport(unsigned char *buf, int size)
{
	int n;

#ifndef __STRICT_ANSI__                       /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
	if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
#else
	if(size>4096)  size = 4096;
#endif

	n = read(Cport,buf, size);

	//printf("%d  %d  %d\n",Cport,n,size);
//	if(n!=-1)
//	{
//		int mmm=0;
//		for(mmm=0;mmm<n;mmm++)
//		printf("%x ",buf[mmm]);
//		printf("\n");
//	}
	return n;
}


int SendByte(unsigned char byte)
{
	int n;

	n = write(Cport, &byte, 1);
	if(n<0)  return(1);

	return(0);
}


int SendBuf(unsigned char *buf, int size)
{
	return(write(Cport, buf, size));
}


void CloseComport()
{
	close(Cport);
	tcsetattr(Cport, TCSANOW, &old_port_settings);
}

/*
Constant  Description
TIOCM_LE  DSR (data set ready/line enable)
TIOCM_DTR DTR (data terminal ready)
TIOCM_RTS RTS (request to send)
TIOCM_ST  Secondary TXD (transmit)
TIOCM_SR  Secondary RXD (receive)
TIOCM_CTS CTS (clear to send)
TIOCM_CAR DCD (data carrier detect)
TIOCM_CD  Synonym for TIOCM_CAR
TIOCM_RNG RNG (ring)
TIOCM_RI  Synonym for TIOCM_RNG
TIOCM_DSR DSR (data set ready)
 */

int IsCTSEnabled(int comport_number)
{
	int status;

	status = ioctl(Cport, TIOCMGET, &status);

	if(status&TIOCM_CTS) return(1);
	else return(0);
}
