#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H
#include <stdint.h>
typedef struct {
 double position_error_m, velocity_error_m_s, com_pitch_rad, com_pitch_rate_rad_s;
 double roll_rad, sample_age_s, yaw_torque_Nm;
} BalanceState;
typedef struct {double left_Nm,right_Nm; int status;} BalanceOutput;
enum {BALANCE_VALID=0,BALANCE_INVALID_SENSOR=1,BALANCE_OUTSIDE_ENVELOPE=2};
BalanceOutput balance_step(const BalanceState *s);
void hoverboard_command(uint8_t out[8],int16_t steer,int16_t speed);
#endif
