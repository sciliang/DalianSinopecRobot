/*
 * rs232.h
 *
 *  Created on: 2019年6月11日
 *      Author: a
 */

#ifndef INCLUDE_RS232_H_
#define INCLUDE_RS232_H_


#ifndef rs232_INCLUDED
#define rs232_INCLUDED

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

//int OpenComport(char *comport, int baudrate,int databits,int parity,int stopbits,int flowcontrol);
int OpenComport(const char *comport, int baudrate);
int PollComport(unsigned char *, int);
int SendByte(unsigned char);
int SendBuf(unsigned char *, int);
void CloseComport();
int IsCTSEnabled();
#endif




#endif /* INCLUDE_RS232_H_ */
