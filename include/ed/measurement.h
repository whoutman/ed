#ifndef measurement_h_
#define measurement_h_

#include "ed/types.h"
#include <geolib/datatypes.h>

namespace ed
{

class Measurement
{

public:

    Measurement();
    Measurement(const geo::Pose3D& sensor_pose, double timestamp, const std::string& type);

    const geo::Pose3D& sensorPose() const { return sensor_pose_; }
    double timestamp() const { return timestamp_; }
    const std::string& type() const { return type_; }

protected:

    geo::Pose3D sensor_pose_;
    double timestamp_;
    std::string type_;

};

}

#endif
