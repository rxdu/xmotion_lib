/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 * Description: 15 by 20 map with sensor range 8, compare heading
 * 
 */

#include <string>
#include <memory>
#include <thread>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.h"
#include "utility/logging/logger.h"
#include "utility/stopwatch/stopwatch.h"
#include "planning/map/map_utils.h"
#include "planning/map/map_config.h"
#include "planning/map/map_info.h"
#include "planning/geometry/graph_builder.h"
#include "planning/geometry/sgrid_builder.h"
#include "quadrotor/path_repair/sim/virtual_quadrotor.h"

using namespace librav;

#define LOOP_PERIOD 500	// ms

int main(int argc, char *argv[])
{
	// set up network first
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
	if (!lcm->good())
	{
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	// init quadrotor planner
	VirtualQuadrotor vquad(lcm);
	vquad.SetMapSize(40,50,5);
	vquad.SetSensorRange(5);
	vquad.SetSensorFOV(M_PI*5.0/9.0);
	vquad.SetInitPosition(Position2Di(19, 0),2);
	vquad.SetGoalPosition(Position2Di(19, 49),2);

	// should not start simulation if configuration is not complete
	if (!vquad.IsReady())
	{
		std::cerr << "ERROR: Incomplete configuration for the simulation" << std::endl;
		return -1;
	}

	stopwatch::StopWatch timer;

	// simulation loop
	while (true)
	{		
		timer.tic();

		vquad.CmpStep();
		lcm->handleTimeout(0);
	
		int64_t duration = LOOP_PERIOD - static_cast<int64_t>(timer.mtoc());
		
		if(duration > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(duration));
	}
}
