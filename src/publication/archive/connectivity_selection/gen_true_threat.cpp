#include <iostream>
#include <memory>

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "stopwatch/stopwatch.h"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

using namespace librav;

int main()
{
    // std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(0, 80, 0, 60);

    // add traffic participant
    GaussianPositionThreat threat_model(0, 0, 1.8/2, 4.8/2);

    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(0, 0, 1, 1);
    participant->threat_func = threat_model;

    // cfield->AddTrafficParticipant(0, participant);

    std::cout << "collision field created" << std::endl;

    Polygon fp;
    fp.AddPoint(0.9, 2.4);
    fp.AddPoint(-0.9, 2.4);
    fp.AddPoint(-0.9, -2.4);
    fp.AddPoint(0.9, -2.4);

    // plot surface
    // LightViz::ShowTrafficParticipantThreat(participant, fp, 15, "true_threat", true);
    // LightViz::ShowCollisionField(cfield);

    return 0;
}