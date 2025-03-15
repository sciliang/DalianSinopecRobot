/*
 * Coordinate_System.h
 *
 *  Created on: 2019年6月11日
 *      Author: a
 */

#ifndef INCLUDE_COORDINATE_SYSTEM_H_
#define INCLUDE_COORDINATE_SYSTEM_H_
#pragma pack(1)

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
	{
		double Next_Longitude;
		double Next_Latitude;
		double velocity;
		double angle_vlocity;
	} an_data_t;

	struct _pid
	{
		float SetSpeed;	   // 设定值
		float ActualSpeed; // 实际值
		float err;		   // 偏差值
		float err_last;	   // 上一次偏差值
		float err_next;	   // 下一次偏差值
		float Kp, Ki, Kd;  // 设定值
		float voltage;	   // 实际转换值
		float integral;	   // 积分累积
		float umax;		   // 偏差上限值
		float umin;		   // 偏差下限值

		// puzzy part
		float err_c;	  // 误差变化率
		float qdetail_kp; // 增量kp对应论域中的值
		float qdetail_ki; // 增量ki对应论域中的值
		float qdetail_kd; // 增量kd对应论域中的值
		float detail_kp;  // 输出增量kp
		float detail_ki;  // 输出增量ki
		float detail_kd;  // 输出增量kd
		float qerror;	  // 输入e对应论域中的值
		float qerror_c;	  // 输入de/dt对应论域中的值
		float e_max;
		float e_min;
		float ec_max;
		float ec_min;
		float kp_max;
		float kp_min;

		float ki_max;
		float ki_min;
		float kd_max;
		float kd_min;

		float e_index_0;
		float e_index_1;
	};

	
	void *controlThread(void);
	void *recvAndSndthread(void);
	void *receive_data(void);
	void Set_min_Velocity(void);
	void Coordinate_System(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INCLUDE_COORDINATE_SYSTEM_H_ */
