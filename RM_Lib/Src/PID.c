/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块
 */

#include "PID.h"
#include "Function.h"

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

void PID_init(PID_TypeDef *pid,uint16_t max_out,float intergrallimit,float deadband,float Kp,float Ki,float Kd,float Kf,float dt){
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergrallimit;
	pid->MaxOut = max_out;
	pid->Target = 0;

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kf = Kf;
	pid->dt = dt;
	pid->ITerm = 0;
}
/***************************PID calculate**********************************/
float PID_Calc(PID_TypeDef *pid, float measure, float target){
	if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE){
			return 0; //Catch ERROR
	}
	pid->Measure = measure;
	pid->Target = target;
	pid->Err = pid->Target - pid->Measure;
  if (ABS(pid->Err) > pid->DeadBand){
		pid->Pout  = pid->Kp * pid->Err;
		pid->Iout += pid->Ki * pid->Err;
		pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err)/pid->dt;
		pid->Fout  = pid->Kf * (pid->Target - pid->Last_Target);

		pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Fout;
		//Output limit
		f_Output_Limit(pid);
	}
	pid->Last_Measure = pid->Measure;
	pid->Last_Target  = pid->Target;
	pid->Last_Output  = pid->Output;
	pid->Last_Err = pid->Err;

	return pid->Output;
}
static void f_Output_Limit(PID_TypeDef *pid){
  if (pid->Output > pid->MaxOut){
    pid->Output = pid->MaxOut;
  }
	if (pid->Output < -(pid->MaxOut)){
    pid->Output = -(pid->MaxOut);
  }
}
void PID_Control(float current, float expected, PID *parameter) {
    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }
        
        limit(parameter->error_inter, parameter->limit, -parameter->limit);
        
        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * (parameter->error_now - parameter->error_last);
}

void PID_Control_Smis(float current, float expected, PID_Smis *parameter, float speed) {
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
    
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }

        limit(parameter->error_inter, parameter->limit, -parameter->limit);

        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * speed;
}
float FeedForward_Calc(FeedForward_Typedef *FF){
    
    FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    
    limit(FF->Out,FF->OutMax,-FF->OutMax);

    return FF->Out;
}
