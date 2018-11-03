#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "threat_field/collision_threat.hpp"

#include "stopwatch/stopwatch.h"
#include "ugvnav_viz/ugvnav_viz.hpp"
#include "location/file_location.hpp"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    TrafficViz::SetupTrafficViz(loader.road_map);

    //////////////////////////////////////////////////

    CovarMatrix2d pos_covar;
    pos_covar << 1, 0,
        0, 1;
    VehicleEstimation veh1({35, 51.2, -10 / 180.0 * M_PI}, 10);
    veh1.SetPositionVariance(pos_covar);
    veh1.SetSpeedVariance(2 * 2);

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels().back();

    std::shared_ptr<CollisionThreat> ct1 = std::make_shared<CollisionThreat>(veh1, ego_chn);
    ct1->PrecomputeParameters(Location::GetDefaultDataFolderPath() + "/reachability/vehicle_threat_combined_state_transition.data");

    std::cout << "------------- all calculation finished -------------" << std::endl;

    // TrafficViz::ShowVehicleCollisionThreat(ct1, "occupancy_estimation");

    return 0;
}