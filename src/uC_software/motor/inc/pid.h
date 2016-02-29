

#ifndef PID_h
#define PID_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stm32f10x.h>



float PID(float P, float I, float input, float ref);

#ifdef __cplusplus
}
#endif


#endif // PID_h

