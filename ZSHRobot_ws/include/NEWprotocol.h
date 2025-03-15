#ifndef NEWPROTOCOL_H_
#define NEWPROTOCOL_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "gsof_records.h"

	typedef struct
	{
		__syscall_slong_t mtype;
		ins_full_navigation_record_t nav992msg2Human_;
	} Nav992MSG2HumanFrame;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* !VAS_COMMON_STRUCT_ENDIAN_H_ */