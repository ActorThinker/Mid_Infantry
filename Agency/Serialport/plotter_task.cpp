#include "cmsis_os.h"
#include "plotter.h"
#include "ins_task.h"


#define Pi 3.14159265358979f
at::Plotter plotter(&huart1);

extern "C" void Plotter_Task(){
	while(true){
		plotter.plot(INS.Roll,INS.Pitch,INS.Yaw);
		osDelay(10);
	}
}