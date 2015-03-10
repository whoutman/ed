#include "ed/measurement.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement() : sensor_pose_(geo::Pose3D(0,0,0)), timestamp_(0)
{

}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(const geo::Pose3D& sensor_pose, double timestamp) : sensor_pose_(sensor_pose), timestamp_(timestamp)
{

}

// ----------------------------------------------------------------------------------------------------

}
