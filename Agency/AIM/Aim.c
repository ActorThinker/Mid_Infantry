#include "Aim.h"
#include "usb_task.h"
#include "Gimbal.h"

void Aim_Shoot(){
	Aim_Data.AimShoot = AimReady;
	if(GimbalCtrl == gAim ){
		 if(ReceiveVisionData.data.dis != -1	&&	ReceiveVisionData.data.FireFlag == 1)
					Aim_Data.AimShoot = AimFire;
	}
}
