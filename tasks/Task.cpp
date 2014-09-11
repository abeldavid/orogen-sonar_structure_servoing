/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/samples/Pointcloud.hpp>
#include <sonar_detectors/SonarDetectorMath.hpp>

using namespace sonar_structure_servoing;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::sonarbeam_featureTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &sonarbeam_feature_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sonar2body;
    if (!_sonar2body.get(ts, sonar2body))
    {
        RTT::log(RTT::Error) << "skip, have no " << _sonar2body.getSourceFrame() << " to " << _sonar2body.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    Eigen::Affine3d body2odometry;
    if (!_body2odometry.get(ts, body2odometry))
    {
        RTT::log(RTT::Error) << "skip, have no " << _body2odometry.getSourceFrame() << " to " << _body2odometry.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    // handle NaN values in translation
    if(!base::isnotnan(sonar2body.translation()))
	sonar2body.translation() = Eigen::Vector3d::Zero();
    if(!base::isnotnan(body2odometry.translation()))
	body2odometry.translation() = Eigen::Vector3d::Zero();
    
    estimator.setAUVInOdometry(body2odometry);
    estimator.updateFeature(sonarbeam_feature_sample, body2odometry * sonar2body);
}

void Task::odometry_samplesTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& odometry_samples_sample)
{
    odometry_time = ts;
    current_orientation = odometry_samples_sample.orientation;
    
    // receive sensor to body transformation
    Eigen::Affine3d sonar2body;
    if (!_sonar2body.get(ts, sonar2body))
    {
        RTT::log(RTT::Error) << "skip, have no " << _sonar2body.getSourceFrame() << " to " << _sonar2body.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    // waiting for target depth
    if((odometry_samples_sample.position.z() > (_fixed_depth.value() * 0.9)) ||
	(odometry_samples_sample.position.z() < (_fixed_depth.value() * 1.1)))
    {
	new_state = WAITING_FOR_TARGET_DEPTH;
	return;
    }
    if(!target_depth_reached)
    {
	target_depth_reached = true;
	estimator.resetCandidateCounter();
    }
    
    if(estimator.isStructureStable())
    {
	new_state = INSPECTING_STRUCTURE;
	
	relative_target_position.x() = estimator.getDistanceToStructure() - _servoing_distance.value();
	relative_target_position.y() = _servoing_speed.value();
	relative_target_position = Eigen::AngleAxisd(_servoing_direction.value(), Eigen::Vector3d::UnitZ()) * relative_target_position;
	
	base::samples::RigidBodyState structure_pose = estimator.getStructurePose();
	base::Vector3d auv_position = odometry_samples_sample.position;
	if(!base::isnotnan(auv_position))
	    auv_position = base::Vector3d::Zero();
	base::Vector3d auv2structure = structure_pose.position - auv_position;
	base::Angle angle_to_structure = base::Angle::fromRad(sonar_detectors::computeRotation(auv2structure.block(0,0,2,1)));
	relative_target_heading = angle_to_structure - base::Angle::fromRad(base::getYaw(odometry_samples_sample.orientation) + _servoing_direction.value());
    }
    else
    {
	new_state = SEARCHING_STRUCTURE;
	if(estimator.getCandidateUpdateCount() >= 1)
	    new_state = VALIDATING_STRUCTURE;
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    estimator.setAngularResolutionSonar(_angular_resolution_sonar.value());
    estimator.setDBScanEpsilon(_dbscan_epsilon.value());
    estimator.setExpectedStructureDiameter(_expected_structure_diameter.value());
    estimator.setMaxMahalanobisDistance(_max_mahalanobis_distance.value());
    estimator.setMinValidCadidates(_min_valid_cadidate_count.value());
    estimator.setProcessNoiseCovariance(0.05 * Eigen::Matrix2d::Identity());
    
    target_depth_reached = false;
    odometry_time.microseconds = 0;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    new_state = RUNNING;
    
    base::Angle left_limit = base::Angle::fromRad(_servoing_direction.value() + _left_opening_angle.value());
    base::Angle right_limit = base::Angle::fromRad(_servoing_direction.value() - _right_opening_angle.value());
    estimator.setEstimationZone(left_limit, right_limit);
    
    relative_target_position.setZero();
    relative_target_heading.rad = 0.0;
    
    TaskBase::updateHook();
    
    if(!odometry_time.isNull())
    {
	// create relative position command
	base::AUVPositionCommand positionCommand;
	positionCommand.x = std::abs(relative_target_position.x()) < 0.001 ? 0.0 : relative_target_position.x();
	positionCommand.y = std::abs(relative_target_position.y()) < 0.001 ? 0.0 : relative_target_position.y();
	positionCommand.z = _fixed_depth.get();
	positionCommand.heading = relative_target_heading.getRad();
	
	//create commnd for aligned 
	base::LinearAngular6DCommand alignedPositionCommand;
	base::LinearAngular6DCommand worldCommand;
	alignedPositionCommand.linear(0) = std::abs(relative_target_position.x()) < 0.001 ? 0.0 : relative_target_position.x();
	alignedPositionCommand.linear(1) = std::abs(relative_target_position.y()) < 0.001 ? 0.0 : relative_target_position.y();
	worldCommand.linear(2) = _fixed_depth.get();
	worldCommand.angular(0) = 0.0;
	worldCommand.angular(1) = 0.0;
	worldCommand.angular(2) = base::Angle::normalizeRad(base::getYaw(current_orientation) + relative_target_heading.getRad());
	
	// write relative position command
	if (_position_command.connected())
	    _position_command.write(positionCommand);
	
	// write aligned position command
	if (_aligned_position_command.connected()){
	    alignedPositionCommand.time = odometry_time;
	    _aligned_position_command.write(alignedPositionCommand);
	}

	if (_world_command.connected()){
	    worldCommand.time = odometry_time;
	    _world_command.write(worldCommand);
	}
    }
    
    // write debug data
    if(_enable_debug.value())
    {
	StructureServoingDebug debug;
	debug.structure_pose = estimator.getStructurePose();
	debug.structure_confidence = estimator.getStructureConfidence();
	debug.cluster_points.time = base::Time::now();
	debug.cluster_points.points = estimator.getClusterPoints();
	debug.candidate_update_count = estimator.getCandidateUpdateCount();
	debug.matrix_mag = estimator.getMagnitude();
	debug.auv_to_structure_line.points = estimator.getAUVToStructure();
	debug.distance_to_structure = estimator.getDistanceToStructure();
	_debug_data.write(debug);
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
