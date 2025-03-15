/****************************************************************/
/*                                                              */
/*          NMEA Packet Protocol Library                        */
/*          C Language Dynamic, Version 1.0                     */
/*   Copyright 2021, Beijing Nuogeng Technology Ltd             */
/*                                                              */
/****************************************************************/
/*
* Copyright (C) 2021 Beijing Nuogeng Technology Ltd
*
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifdef __cplusplus
extern "C"
{
#endif

#define NMEA_PACKET_HEADER_SIZE 6
#define RECV_BUFFER_SIZE 2048

// point to address which incoming data can be append here
#define nmea_decoder_pointer(nmea_decoder) &(nmea_decoder)->recv_buffer[(nmea_decoder)->recv_buf_length]
// size have not filled data
#define nmea_decoder_size(nmea_decoder) (sizeof((nmea_decoder)->recv_buffer) - (nmea_decoder)->recv_buf_length)
// update filled data size
#define nmea_decoder_increment(nmea_decoder, bytes_received) (nmea_decoder)->recv_buf_length += bytes_received


typedef struct
{
  uint8_t header[NMEA_PACKET_HEADER_SIZE];
  uint8_t length;
  uint8_t* data;
} nmea_record_t;

typedef struct
{
  uint8_t header[NMEA_PACKET_HEADER_SIZE];
  uint8_t length;
  uint8_t data[20];
} nmea_record_t_new;

typedef struct
{
  uint8_t recv_buffer[RECV_BUFFER_SIZE];
  int16_t recv_buf_length;
  int16_t decode_iterator;
} nmea_decoder_t;


const char* toSystemStatusString(int system_status);
const char* toRtkString(int rtk_status);

int nmea_record_decode(nmea_decoder_t* nmea_decoder, nmea_record_t* nmea_record);

#ifdef __cplusplus
}
#endif