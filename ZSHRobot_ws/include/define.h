/*
 * define.h
 *
 *  Created on: 2019年9月10日
 *      Author: superman
 */

#ifndef INCLUDE_DEFINE_H_
#define INCLUDE_DEFINE_H_

#ifdef __cplusplus
extern "C"
{
#endif
	// 机器人控制模式
	enum
	{
		ROBWAIT, // 待机
		ROBTElE, // 遥控
		ROBAUTO, // 自主
		ROBNO_START, // 自主
	};
	// 机器人控制模式
	enum
	{
		TracePause = 1, // 暂停
		TraceStop,		// 终止
		TraceContinue,	// 继续
	};

	// 机器人档位
	// 参数：1对应N , 2 L, 3 H, 4 R, 5 P!
	enum
	{
		INVALID_GEAR, // 无效挡位
		NEUTRAL_GEAR, // 空挡
		LOW_GEAR,	  // 低速挡
		HIGH_GEAR,	  // 高速挡
		REVERSE_GEAR, // 倒挡
		PARKING_GEAR, // 驻车
	};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INCLUDE_DEFINE_H_ */
