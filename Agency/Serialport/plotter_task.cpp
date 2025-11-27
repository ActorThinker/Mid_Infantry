#include "cmsis_os.h"
#include "plotter.h"
#include "ins_task.h"

at::Plotter plotter(&huart1);

extern "C" void Plotter_Task(){
	while(true){
		plotter.plot(INS.Roll,INS.Pitch,INS.Yaw);
		osDelay(10);
	}
}