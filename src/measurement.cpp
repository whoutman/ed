#include "ed/measurement.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement() : sensor_pose_(geo::Pose3D(0,0,0)), timestamp_(0), type_("")
{

}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(const geo::Pose3D& sensor_pose, double timestamp, const std::string& type) : sensor_pose_(sensor_pose), timestamp_(timestamp), type_(type)
{

}

// ----------------------------------------------------------------------------------------------------

}
