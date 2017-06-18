/*
 * quadsim_hbird_pos_step.cpp
 *
 *  Created on: Nov 1, 2016
 *      Author: rdu
 */

#include <quad_hbird_sim/quad_hbird_sim_controller.h>
#include <iostream>
#include <memory>
#include <cmath>

#include "vrep_sim/vrep_interface/robot_sim_process.h"

#include "quad_hbird_sim/quad_hbird_sim_client.h"
#include "quad_hbird_sim/quad_hbird_sim_controller.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	std::shared_ptr<QuadHbirdSimClient> client = std::make_shared<QuadHbirdSimClient>();
	std::shared_ptr<QuadHbirdSimController> controller = std::make_shared<QuadHbirdSimController>();

	// set quadrotor init pose
	//controller->SetInitPose(-1.8,2,0.5,-M_PI/4);
	controller->SetInitPose(-1.8,1.8,0.8,M_PI/4);
	controller->BroadcastRobotState(true);
	char* home_path;
	home_path = getenv ("HOME");
	std::string log_path;
	if (home_path!=NULL)
	{
		std::string hm(home_path);
		log_path = hm+"/Workspace/srcl_rtk/srcl_ctrl/pc/control/log/quad";
	}
	else
	{
		// default path
		log_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/control/log/quad";
	}
	controller->InitLogger("quadsim_hummingbird", log_path);//controller->SetMotionMode(MotionMode::POS_STEP_RESPONSE);

	// create a simulation process
	RobotSimProcess<DataFromQuadSim, DataToQuadSim,QuadState, QuadCmd> process(client,controller);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 1;
}


