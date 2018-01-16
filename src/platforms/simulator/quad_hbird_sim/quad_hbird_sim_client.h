/* 
 * quad_hbird_sim_client.h
 * 
 * Created on: Sep 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef QUAD_HBIRD_SIM_CLIENT_H
#define QUAD_HBIRD_SIM_CLIENT_H

#include <cstdint>

#include "common/librav_types.hpp"

#include "quad_hbird_sim/quad_hbird_sim_types.h"
#include "vrep_sim/vrep_interface/robot_sim_client.h"

namespace librav {

class QuadHbirdSimClient : public RobotSimClient<DataFromQuadSim, DataToQuadSim>
{
public:
	QuadHbirdSimClient();
	QuadHbirdSimClient(simxInt clientId);
	~QuadHbirdSimClient();

private:
	virtual void ConfigDataStreaming(void);

public:
	virtual bool ReceiveDataFromRobot(DataFromQuadSim& rstate);
	virtual void SendDataToRobot(const DataToQuadSim& rcmd);

private:
	bool ReceiveGyroData(Point3f& data);
	bool ReceiveAccData(Point3f& data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);
	bool Get3DScanPoints(std::vector<Point3f>& points);

	bool ReceiveQuadPosition(Point3f& data);
	bool ReceiveQuadVelocity(Point3f& data);
	bool ReceiveQuadOrientation(Point3f& data);
	bool ReceiveQuadQuaternion(Quaternion& data);

private:
	const uint64_t max_motor_speed_;

private:
	simxInt quad_handle_;
	simxInt ref_handle_;

private:
	// quadrotor kinematics/dynamics
	IMUData imu_data;
	simxFloat quad_pos[3];
	simxFloat quad_linear_vel[3];
	simxFloat quad_angular_vel[3];
	simxFloat quad_ori[3];
	simxUChar* gyro_sig;
	simxInt gyro_sig_size;
	simxUChar* acc_sig;
	simxInt acc_sig_size;
	simxUChar* quat_sig;
	simxInt quat_sig_size;
	simxUChar* scannerptr_sig;
	simxInt scannerptr_sig_size;

	// vision
	simxInt camera_handle_;
	simxUChar *image_raw_;
	int img_res[2];
};

}

#endif /* QUAD_HBIRD_SIM_CLIENT_H */