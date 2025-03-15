/****************************************************************/
/*                                                              */
/*          GSOF Packet Protocol Library                        */
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
#ifndef GSOFRECORDS_H_
#define GSOFRECORDS_H_
#pragma pack(1)
#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	record_type_utc = 0x10,
	record_type_ins_full_navigation = 0x31,
	record_type_ins_rms_info = 0x32,
} record_type_e;

typedef struct
{
	uint8_t record_type; // =0x10
	uint8_t record_length; // = 0x09
	uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
	uint16_t gps_week; // GPS week count since January 1980
	uint16_t utc_offset; // GPS to UTC offset
	union
	{
		uint8_t r_uint8;
		struct
		{
			uint8_t time_valid : 1; // 1 = week and millisecond of week valid
			uint8_t offset_valid : 1; // 1 = UTC offset validity
			uint8_t recv1 : 6;
		} val;
	} flags;

	//stream_size = 11(record_length+2);
} utc_record_t;

enum QualityIndication
{
	Quality_FixInvalid = 0,
	Quality_Autonomous,
	Quality_Differential,
	Quality_PPSMode,
	Quality_RTKFixed,
	Quality_RTKFloat,
	Quality_DeadReckoning,
	Quality_ManualInputMode,
	Quality_SimulatorMode
};

enum AlignStatus
{
	Align_GpsOnly = 0,
	Align_CoarseLeveling,
	Align_DegradedSolution,
	Align_Aligned,
	Align_FullNavigationMode
};

//=========================== ins_full_navigation_record ===========================//
typedef struct
{
	uint8_t record_type; // =0x31
	uint8_t record_length; // = 0x68
	uint16_t gps_week; // GPS week count since January 1980
	uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
	uint8_t align_status;
	uint8_t quality_indication;
	double latitude; // deg
	double longitude; // deg
	double altitude; // meter
	float north_velocity; // m/s
	float east_velocity; // m/s
	float down_velocity; // m/s
	float total_speed; // m/s
	double roll; // deg
	double pitch; // deg
	double heading; // deg
	double track_angle; // deg
	float angle_rate_x; // deg/s
	float angle_rate_y; // deg/s
	float angle_rate_z; // deg/s
	float longitudinal_acceleration; // m/s2
	float traverse_acceleration; // m/s2
	float down_acceleration; // m/s2

	//stream_size = 106(record_length+2);
} ins_full_navigation_record_t;

typedef struct
{
	uint8_t record_type; // =0x32
	uint8_t record_length; // = 0x2C
	uint16_t gps_week; // GPS week count since January 1980
	uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
	uint8_t align_status;
	uint8_t quality_indication;
	float north_position_rms; // m
	float east_position_rms; // m
	float down_position_rms; // m
	float north_velocity_rms; // m/s
	float east_velocity_rms; // m/s
	float down_velocity_rms; // m/s
	float roll_rms; // degree
	float pitch_rms; // degree
	float heading_rms; // degree

	//stream_size = 46(record_length+2);
} ins_rms_info_record_t;


// int decode_utc_record(utc_record_t *utc_record, gsof_record_t *gsof_record);
// int decode_ins_full_navigation_record(ins_full_navigation_record_t *ins_full_navigation_record, gsof_record_t *gsof_record);
// int decode_ins_rms_info_record(ins_rms_info_record_t *ins_rms_info_record, gsof_record_t *gsof_record);

#ifdef __cplusplus
}
#endif
#endif 
#pragma pack()