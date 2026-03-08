#include "cmsis_os.h"
#include "plotter.h"
#include "ins_task.h"
#include "Gimbal.h"

at::Plotter plotter(&huart1);

extern "C" void Plotter_Task(){
	while(true){
		plotter.plot(IMU.Angle_Yawcontinuous,IMU.VisionAngle);
		osDelay(10);
	}
}