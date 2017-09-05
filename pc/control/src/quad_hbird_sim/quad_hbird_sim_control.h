/*
 * quad_sim_control.h
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_HBIRD_SIM_QUAD_HBIRD_SIM_CONTROL_H_
#define CONTROL_SRC_QUAD_HBIRD_SIM_QUAD_HBIRD_SIM_CONTROL_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/srcl_ctrl.hpp"

#include "vrep_sim/vrep_interface/robot_sim_control.h"
#include "quad_ctrl/controller/att_quat_con.h"
#include "quad_ctrl/controller/pos_quat_con.h"
#include "quad_ctrl/motion_server/motion_server.h"
#include "quad_ctrl/data_trans/quad_data_transmitter.h"
#include "quad_ctrl/data_types/quad_sim_types.h"
#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl {

class QuadHbirdSimControl: public RobotSimControl<DataFromQuadSim, DataToQuadSim,QuadState, QuadCmd>
{
public:
	QuadHbirdSimControl();

private:
	UAVTrajectoryPoint previous_state_;

	std::unique_ptr<AttQuatCon> att_quat_con_;
	std::unique_ptr<PosQuatCon> pos_quat_con_;

	std::shared_ptr<lcm::LCM> lcm_;

	std::shared_ptr<MotionServer> motion_server_;

	bool broadcast_rs_;
	std::shared_ptr<QuadDataTransmitter> data_trans_;

public:
	virtual DataToQuadSim ConvertRobotCmdToSimCmd(const QuadCmd& cmd);
	virtual void UpdateRobotState(const DataFromQuadSim& data);
	virtual QuadCmd UpdateCtrlLoop();

public:
	void SetInitPose(float x, float y, float z, float yaw);
	void BroadcastRobotState(bool cmd) { broadcast_rs_ = cmd; };
	void InitLogger(std::string log_name_prefix, std::string log_save_path);
	void SetMotionMode(MotionMode mode) { motion_server_->SetMotionMode(mode); };
};

}

#endif /* CONTROL_SRC_QUAD_HBIRD_SIM_QUAD_HBIRD_SIM_CONTROL_H_ */