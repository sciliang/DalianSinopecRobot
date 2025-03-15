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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "nmea_packet_protocol.h"

const char *toSystemStatusString(int system_status)
{
  /*0 初始化
   * 1 卫导模式
   * 2 组合导航模式
   * 3 纯惯导模式
   */
  switch (system_status)
  {
  case 0:
    return "Init";
  case 1:
    return "GNSS";
  case 2:
    return "GNSS_INS";
  case 3:
    return "INS";
  default:
    break;
  }
  return "Unknown";
}

const char *toRtkString(int rtk_status)
{
  /*0：不定位不定向
  1：单点定位定向
  2：伪距差分定位定向
  3：组合推算
  4：RTK稳定解定位定向
  5：RTK浮点解定位定向
  6：单点定位不定向
  7：伪距差分定位不定向
  8：RTK 稳定解定位不定向
  9：RTK浮点解定位不定向
  */
  switch (rtk_status)
  {
  case 0:
    return "None";
  case 1:
    return "SPS_DualAnt";
  case 2:
    return "DGNSS_DualAnt";
  case 3:
    return "DeadReckoning";
  case 4:
    return "RtkInteger_DualAnt";
  case 5:
    return "RtkFloat_DualAnt";
  case 6:
    return "SPS_SingleAnt";
  case 7:
    return "DGNSS_SingleAnt";
  case 8:
    return "RtkInteger_SingleAnt";
  case 9:
    return "RtkFloat_SingleAnt";
  default:
    break;
  }
  return "Unknown";
}

/*
 * Function to decode nmea_records from raw data
 * Returns 0 with valid nmea_record
 */
int nmea_record_decode(nmea_decoder_t *nmea_decoder, nmea_record_t *nmea_record)
{
  // 检查缓存内容，中间可能包含一个或者多个nmea_record
  while (nmea_decoder->decode_iterator + NMEA_PACKET_HEADER_SIZE <= nmea_decoder->recv_buf_length)
  {
    uint8_t *recv_buf = nmea_decoder->recv_buffer + nmea_decoder->decode_iterator;
    // 查找起始'$'符
    if (recv_buf[0] != '$')
    {
      ++nmea_decoder->decode_iterator;
      continue;
    }

    // 检查是否够一个完整帧，4='*'之后的字符串"ff\r\n"
    for (int end = 0; end < nmea_decoder->recv_buf_length - nmea_decoder->decode_iterator; ++end)
    {
      // 查找结尾0x0D'\r', 0x0A'\n'符
      if (recv_buf[end] == 0x0A && (end > 1 && recv_buf[end - 1] == 0x0D))
      {
        memcpy(nmea_record->header, recv_buf, sizeof(nmea_record->header));
        nmea_record->length = end + 1;
        nmea_record->data = recv_buf;
        nmea_decoder->decode_iterator += end + 1;
        return 0; // 返回一个有效的数据包nmea_record
      }
    }

    // 程序运行到这里，说明找到了起始字符'$'，但是没有找到结尾字符'\r\n'，不够一个完整帧
    break;
  }

  // 判断缓存中是否仍然有未检查的数据
  if (nmea_decoder->decode_iterator < nmea_decoder->recv_buf_length)
  {
    if (nmea_decoder->decode_iterator > 0)
    { // 拷贝未检查的数据至recv_buffer起始位置
      memmove(nmea_decoder->recv_buffer,
              nmea_decoder->recv_buffer + nmea_decoder->decode_iterator,
              (nmea_decoder->recv_buf_length - nmea_decoder->decode_iterator) * sizeof(uint8_t));

      nmea_decoder->recv_buf_length -= nmea_decoder->decode_iterator;
    }
  }
  else
    nmea_decoder->recv_buf_length = 0;

  // 复位解码位置至recv_buffer起始位置
  nmea_decoder->decode_iterator = 0;
  return 1;
}