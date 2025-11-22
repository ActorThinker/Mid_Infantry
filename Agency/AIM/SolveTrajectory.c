#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"
//#include "arm_math.h"
struct SolveTrajectoryParams st;
struct tar_pos tar_position[4];
float t = 0.5f; 

float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
	
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    if(t < 0)
    {
        t = 0;
        return 0;
    }
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}

float completeAirResistanceModel(float s, float v, float angle)

{

    return 0;

}

float DZ,ANGLE;
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 40; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
			  t = s / (v * cos(angle_pitch));
        z_actual = v * sin(angle_pitch) * t - GRAVITY * t * t / 2;
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.08f*(z - z_actual);
        DZ = dz;
				z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
		ANGLE = angle_pitch; 
    return angle_pitch;
}
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

    float timeDelay = st.bias_time/1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;
	int use_1 = 1;
	int i = 0;
    int idx = 0; 
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }

    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2; 
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

	

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    float temp_pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}
