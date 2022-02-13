#include "imm_motion_predict/imm_motion_predict.h"

ImmMotionPredict::ImmMotionPredict() : nh_(), private_nh_("~"), MAX_PREDICTION_SCORE_(1.0)
{
    private_nh_.param<double>("interval_sec", interval_sec_, 0.1);
    private_nh_.param<int>("num_prediction", num_prediction_, 10);

    predicted_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/prediction/motion_predictor/objects", 1);
    predicted_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prediction/motion_predictor/path_markers", 1);
    detected_objects_sub_ = nh_.subscribe("/cross_cov/tracker/objects", 1, &ImmMotionPredict::objectsCallback, this);
}

ImmMotionPredict::~ImmMotionPredict() {}

void ImmMotionPredict::initializeROSmarker(const std_msgs::Header &header,
                                           const geometry_msgs::Point &position,
                                           const int object_id,
                                           visualization_msgs::Marker &predicted_line)
{
    predicted_line.lifetime = ros::Duration(0.2);
    predicted_line.header.frame_id = header.frame_id;
    predicted_line.header.stamp = header.stamp;
    predicted_line.ns = "predicted_line";
    predicted_line.action = visualization_msgs::Marker::ADD;
    predicted_line.pose.orientation.w = 1.0;
    predicted_line.id = object_id;
    predicted_line.type = visualization_msgs::Marker::LINE_STRIP;
    predicted_line.scale.x = 2;

    // Points are green
    predicted_line.color.r = 1.0f;
    predicted_line.color.a = 0.5;

    geometry_msgs::Point p;
    p.x = position.x;
    p.y = position.y;
    p.z = 0;
    predicted_line.points.push_back(p);
}

void ImmMotionPredict::makePrediction(autoware_msgs::DetectedObject &object, visualization_msgs::Marker &predicted_line)
{
    autoware_msgs::DetectedObject target_object = object;
    autoware_msgs::Lane predicted_trajectory;
    // target_object.score = MAX_PREDICTION_SCORE_;
    initializeROSmarker(target_object.header, target_object.pose.position, target_object.id, predicted_line);
    
    for (int ith_prediction = 0; ith_prediction < num_prediction_; ith_prediction++) {
        autoware_msgs::DetectedObject predicted_object = generatePredictedObject(target_object);
        // predicted_object.score = (-1 / num_prediction_) * ith_prediction + MAX_PREDICTION_SCORE_;
        // predicted_objects_vec.push_back(predicted_object);
        target_object = predicted_object;

        geometry_msgs::Point p;
        p.x = predicted_object.pose.position.x;
        p.y = predicted_object.pose.position.y;
        p.z = 0;
        predicted_line.points.push_back(p);

        autoware_msgs::Waypoint predicted_waypoint;
        predicted_waypoint.pose.pose = predicted_object.pose;
        predicted_trajectory.waypoints.push_back(predicted_waypoint);
    }
    object.candidate_trajectories.lanes.push_back(predicted_trajectory);
}

/*
This package is a template package for more sopisticated prediction packages.
Feel free to change/modify generatePredictedObject function
and send pull request to Autoware
 */

autoware_msgs::DetectedObject ImmMotionPredict::generatePredictedObject(const autoware_msgs::DetectedObject &object)
{
    autoware_msgs::DetectedObject predicted_object;
    if (object.behavior_state == MotionModel::CV) {
        predicted_object = moveConstantVelocity(object);
    } else if (object.behavior_state == MotionModel::CTRV) {
        predicted_object = moveConstantTurnRateVelocity(object);
    } else {
        // This is because random motion's velocity is 0
        predicted_object = object;
    }
    return predicted_object;
}

geometry_msgs::PolygonStamped ImmMotionPredict::getPredictedConvexHull(const geometry_msgs::PolygonStamped &in_polygon,
                                                                       const double delta_x,
                                                                       const double delta_y)
{
    geometry_msgs::PolygonStamped out_polygon;
    out_polygon.header = in_polygon.header;
    for (auto point : in_polygon.polygon.points) {
        geometry_msgs::Point32 out_point;
        out_point.x = point.x + delta_x;
        out_point.y = point.y + delta_y;
        out_point.z = point.z;
        out_polygon.polygon.points.push_back(out_point);
    }
    return out_polygon;
}


autoware_msgs::DetectedObject ImmMotionPredict::moveConstantVelocity(const autoware_msgs::DetectedObject &object)
{
    autoware_msgs::DetectedObject predicted_object;
    predicted_object = object;
    double px = object.pose.position.x;
    double py = object.pose.position.y;
    double velocity = object.velocity.linear.x;
    double yaw = generateYawFromQuaternion(object.pose.orientation);

    double delta_x = velocity * cos(yaw) * interval_sec_;
    double delta_y = velocity * sin(yaw) * interval_sec_;

    // predicted state values
    double prediction_px = px + delta_x;
    double prediction_py = py + delta_y;

    predicted_object.pose.position.x = prediction_px;
    predicted_object.pose.position.y = prediction_py;

    predicted_object.convex_hull = getPredictedConvexHull(object.convex_hull, delta_x, delta_y);  // unread by Kenny

    return predicted_object;
}

autoware_msgs::DetectedObject ImmMotionPredict::moveConstantTurnRateVelocity(const autoware_msgs::DetectedObject &object)
{
    autoware_msgs::DetectedObject predicted_object;
    predicted_object = object;
    double px = object.pose.position.x;
    double py = object.pose.position.y;
    double velocity = object.velocity.linear.x;
    double yaw = generateYawFromQuaternion(object.pose.orientation);
    double yawd = object.acceleration.linear.y;

    // predicted state values
    double prediction_px, prediction_py, delta_x, delta_y;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        delta_x = velocity / yawd * (sin(yaw + yawd * interval_sec_) - sin(yaw));
        delta_y = velocity / yawd * (cos(yaw) - cos(yaw + yawd * interval_sec_));
        prediction_px = px + delta_x;
        prediction_py = py + delta_y;
    } else {
        delta_x = velocity * interval_sec_ * cos(yaw);
        delta_y = velocity * interval_sec_ * sin(yaw);
        prediction_px = px + delta_x;
        prediction_py = py + delta_y;
    }
    double prediction_yaw = yaw + yawd * interval_sec_;

    while (prediction_yaw > M_PI)
        prediction_yaw -= 2. * M_PI;
    while (prediction_yaw < -M_PI)
        prediction_yaw += 2. * M_PI;

    predicted_object.pose.position.x = prediction_px;
    predicted_object.pose.position.y = prediction_py;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, prediction_yaw);
    predicted_object.pose.orientation.x = q[0];
    predicted_object.pose.orientation.y = q[1];
    predicted_object.pose.orientation.z = q[2];
    predicted_object.pose.orientation.w = q[3];

    predicted_object.convex_hull = getPredictedConvexHull(object.convex_hull, delta_x, delta_y);

    return predicted_object;
}

double ImmMotionPredict::generateYawFromQuaternion(const geometry_msgs::Quaternion &quaternion)
{
    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void ImmMotionPredict::objectsCallback(const autoware_msgs::DetectedObjectArray &input)
{
    autoware_msgs::DetectedObjectArray output;
    output.header = input.header;
    visualization_msgs::MarkerArray predicted_lines;

    for (const auto &object : input.objects) {
        autoware_msgs::DetectedObject predicted_objects = object;
        visualization_msgs::Marker predicted_line;
        if (isObjectValid(predicted_objects)) {
            makePrediction(predicted_objects, predicted_line);

            // concate to output object array
            // output.objects.insert(output.objects.end(), predicted_objects_vec.begin(), predicted_objects_vec.end());
            output.objects.push_back(predicted_objects);

            predicted_lines.markers.push_back(predicted_line);
        }
    }
    predicted_objects_pub_.publish(output);
    predicted_paths_pub_.publish(predicted_lines);
}

bool ImmMotionPredict::isObjectValid(const autoware_msgs::DetectedObject &in_object)
{
    if (!in_object.valid || std::isnan(in_object.pose.orientation.x) || std::isnan(in_object.pose.orientation.y) ||
        std::isnan(in_object.pose.orientation.z) || std::isnan(in_object.pose.orientation.w) || std::isnan(in_object.pose.position.x) ||
        std::isnan(in_object.pose.position.y) || std::isnan(in_object.pose.position.z) || (in_object.dimensions.x <= 0) ||
        (in_object.dimensions.y <= 0) || !in_object.pose_reliable || (in_object.dimensions.z <= 0)) {
        return false;
    }
    return true;
}
