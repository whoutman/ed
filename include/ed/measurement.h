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
    Measurement(const geo::Pose3D& sensor_pose, double timestamp);

    const geo::Pose3D& sensorPose() const { return sensor_pose_; }
    double timestamp() const { return timestamp_; }
    const virtual std::string type() const { return ""; }

protected:

    geo::Pose3D sensor_pose_;
    double timestamp_;

};

}

#endif
