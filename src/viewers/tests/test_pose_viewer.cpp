#include <iostream>

#include "viewers/pose_viewer.hpp"

using namespace librav;

int main()
{
    PoseViewer viewer;

    viewer.SetupViewer();
    viewer.Start();

    return 0;
}