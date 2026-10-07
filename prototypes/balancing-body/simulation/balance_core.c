/* Simulation-reviewed portable core, not a flashed motor driver.
 * COM attitude comes from fused IMU plus source-model forward kinematics.
 * Runtime torque-to-current calibration and scheduling belong to hardware port.
 * Nonzero status requests mechanical catching/parking; zero Nm is NOT a safe stand.
 */
#include "balance_core.h"
#include "controller_gains.h"
#include <math.h>
static double clip(double x,double lo,double hi){return fmax(lo,fmin(hi,x));}
BalanceOutput balance_step(const BalanceState *s){
 BalanceOutput o={0,0,BALANCE_VALID};
 if(!isfinite(s->position_error_m)||!isfinite(s->velocity_error_m_s)||
    !isfinite(s->com_pitch_rad)||!isfinite(s->com_pitch_rate_rad_s)||
    !isfinite(s->roll_rad)||!isfinite(s->sample_age_s)||!isfinite(s->yaw_torque_Nm)||
    s->sample_age_s<0||s->sample_age_s>.020){o.status=BALANCE_INVALID_SENSOR;return o;}
 /* Deliberately narrower candidate operating envelope than demonstrated recovery. */
 if(fabs(s->com_pitch_rad)>.1745329252||fabs(s->roll_rad)>.0872664626){o.status=BALANCE_OUTSIDE_ENVELOPE;return o;}
 double u=BALANCE_K_0*s->position_error_m+BALANCE_K_1*s->velocity_error_m_s+
          BALANCE_K_2*s->com_pitch_rad+BALANCE_K_3*s->com_pitch_rate_rad_s;
 double common=clip(u/2,-BALANCE_TORQUE_EACH_MAX,BALANCE_TORQUE_EACH_MAX);
 /* Reserve saturation headroom for balance; yaw uses only what remains. */
 double differential=clip(s->yaw_torque_Nm,-BALANCE_TORQUE_EACH_MAX+fabs(common),BALANCE_TORQUE_EACH_MAX-fabs(common));
 o.left_Nm=common+differential;o.right_Nm=common-differential;return o;
}
void hoverboard_command(uint8_t out[8],int16_t steer,int16_t speed){
 uint16_t words[4]={0xABCD,(uint16_t)steer,(uint16_t)speed,(uint16_t)(0xABCD^(uint16_t)steer^(uint16_t)speed)};
 for(int i=0;i<4;i++){out[2*i]=(uint8_t)words[i];out[2*i+1]=(uint8_t)(words[i]>>8);}
}
