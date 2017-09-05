/*
 * test_rrts.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <iostream>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "rrtstar/rrtstar_kd.h"

using namespace ompl;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void test_callback(const ob::Planner* planner, const std::vector<const base::State*> & states, const ob::Cost cost)
{
	std::cout << "callback called" << std::endl;
	ob::PlannerData status_data(planner->getSpaceInformation());
	planner->getPlannerData(status_data);

	std::cout << "number of vertices: " << status_data.numVertices() << std::endl;
}

int main(int argc, char** argv)
{
	// define the R3 space for (x,y,z)
	ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(-10);
	bounds.setHigh(10);
	r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// define the SO(2) space for (yaw)
	ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

	// define the R2 space for testing (x,y)
	ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
	ompl::base::RealVectorBounds bounds2(2);
	bounds2.setLow(0);
	bounds2.setHigh(1.5);
	r2->as<ob::RealVectorStateSpace>()->setBounds(bounds2);

	// add two spaces to get a R3*SO(2) space
//	ompl::base::StateSpacePtr flat_space = r3 + so2;
	ompl::base::StateSpacePtr flat_space = r2;

	// create an instance of planner
	ob::SpaceInformationPtr si(new ob::SpaceInformation(flat_space));
	auto rrt_planner =  new RRTStarKD(si);
	// auto rrt_planner =  new og::RRTstar(si);
	rrt_planner->setRange(0.01);
	rrt_planner->EnableIterationData(true);
	base::PlannerPtr planner(rrt_planner);

	// define the problem to be solved
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
	pdef->setIntermediateSolutionCallback(test_callback);

	ompl::base::ScopedState<> start(flat_space);
	start.random();
//	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
//	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;
	ob::ScopedState<> goal(flat_space);
	goal.random();
//	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
//	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

	std::cout << "\n Start: " << std::endl;
	std::cout << start;

	std::cout << "\n Goal: " << std::endl;
	std::cout << goal;

	pdef->setStartAndGoalStates(start, goal);

	// apply configurations to planner
	planner->setProblemDefinition(pdef);
	planner->setup();

//	// print the settings for this space
//	std::cout << "----------------------" << std::endl;
//	si->printSettings(std::cout);
//
//	// print the problem settings
//	std::cout << "----------------------" << std::endl;
//	pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(0.5);
	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution" << std::endl;

		// print the path to screen
//		path->print(std::cout);

//		std::ofstream outFile("output.txt");
//		dynamic_cast<const og::PathGeometric&>(*path).printAsMatrix(std::cout);
//		path->as<og::PathGeometric>()->printAsMatrix(outFile);
		std::vector<base::State*> states = path->as<og::PathGeometric>()->getStates();
	}
	else
		std::cout << "No solution found" << std::endl;

	planner->clear();
}


