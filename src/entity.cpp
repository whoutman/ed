#include "ed/entity.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/measurement.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(const UUID& id, const TYPE& type, const unsigned int& measurement_buffer_size) :
    id_(id),
    type_(type),
    shape_revision_(0),
    measurements_(measurement_buffer_size),
    convex_hull_buffer_(20),
    measurements_seq_(0),
    pose_(geo::Pose3D::identity()),
    velocity_(geo::Pose3D::identity())
{
    convex_hull_.center_point = geo::Vector3(0,0,0);
}

// ----------------------------------------------------------------------------------------------------

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void Entity::setShape(const geo::ShapeConstPtr& shape)
{
    if (shape_ != shape)
    {
        ++shape_revision_;
        shape_ = shape;

        // ----- Calculate convex hull -----

        const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

        if (!vertices.empty())
        {
            geo::Vector3 p_total(0, 0, 0);

            cv::Mat_<cv::Vec2f> pointMat(1, vertices.size());
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            point_cloud.resize(vertices.size());

            convex_hull_.min_z = vertices[0].z;
            convex_hull_.max_z = vertices[0].z;

            for(unsigned int i = 0; i < vertices.size(); ++i)
            {
                geo::Vector3 p_MAP = pose_ * vertices[i];
                convex_hull_.min_z = std::min(convex_hull_.min_z, p_MAP.z);
                convex_hull_.max_z = std::max(convex_hull_.max_z, p_MAP.z);

                pointMat(0, i) = cv::Vec2f(p_MAP.x, p_MAP.y);
                point_cloud.points[i] = pcl::PointXYZ(p_MAP.x, p_MAP.y, p_MAP.z);

                p_total += p_MAP;
            }

            std::vector<int> chull_mask_indices;
            cv::convexHull(pointMat, chull_mask_indices);

            convex_hull_.chull.resize(chull_mask_indices.size());
            for(unsigned int i = 0; i < chull_mask_indices.size(); ++i)
                convex_hull_.chull[i] = point_cloud.points[chull_mask_indices[i]];

            convex_hull_.center_point = p_total / vertices.size();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::addMeasurement(MeasurementConstPtr measurement)
{
    // Push back the measurement
    measurements_.push_front(measurement);
    measurements_seq_++;
}

// ----------------------------------------------------------------------------------------------------

void Entity::measurements(std::vector<MeasurementConstPtr>& measurements, double min_timestamp) const
{
    for(boost::circular_buffer<MeasurementConstPtr>::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    {
        const MeasurementConstPtr& m = *it;
        if (m->timestamp() > min_timestamp)
            measurements.push_back(m);
    }
}


// ----------------------------------------------------------------------------------------------------

void Entity::measurements(std::vector<MeasurementConstPtr>& measurements, unsigned int num) const
{
    for(unsigned int i = 0; i < num && i < measurements_.size(); ++i)
    {
        measurements.push_back(measurements_[i]);
    }
}

// ----------------------------------------------------------------------------------------------------

MeasurementConstPtr Entity::lastMeasurement() const
{
    if (measurements_.empty())
        return MeasurementConstPtr();

    return measurements_.front();
}

// ----------------------------------------------------------------------------------------------------

UUID Entity::generateID() {
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string s;
    for (int i = 0; i < 32; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        s += alphanum[n];
    }

    return UUID(s);
}

}
