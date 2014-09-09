#ifndef sonar_structure_servoing_TYPES_HPP
#define sonar_structure_servoing_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>

namespace sonar_structure_servoing
{

struct StructureServoingDebug
{
    base::samples::RigidBodyState structure_pose;
    base::samples::RigidBodyState structure_confidence;
    base::samples::Pointcloud cluster_points;
    unsigned candidate_update_count;
    double matrix_mag;
    base::samples::Pointcloud auv_to_structure_line;
    double distance_to_structure;
};

}

#endif

