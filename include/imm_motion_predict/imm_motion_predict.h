#ifndef NAIVE_MOTION_PREDICT_H
#define NAIVE_MOTION_PREDICT_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

enum MotionModel : int {
    CV = 0,    // constant velocity
    CTRV = 1,  // constant turn rate and velocity
    RM = 2,    // random motion
};

class ImmMotionPredict
{
private:
    // nodehandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // max prediction score
    const double MAX_PREDICTION_SCORE_;

    // ros publisher
    ros::Publisher predicted_objects_pub_;
    ros::Publisher predicted_paths_pub_;

    // ros Subscriber
    ros::Subscriber detected_objects_sub_;

    // prediction param
    double interval_sec_;
    int num_prediction_;
    double sensor_height_;
    double filter_out_close_object_threshold_;

    void objectsCallback(const autoware_msgs::DetectedObjectArray &input);

    void initializeROSmarker(const std_msgs::Header &header,
                             const geometry_msgs::Point &position,
                             const int object_id,
                             visualization_msgs::Marker &predicted_line);

    void makePrediction(const autoware_msgs::DetectedObject &object,
                        std::vector<autoware_msgs::DetectedObject> &predicted_objects,
                        visualization_msgs::Marker &predicted_line);

    autoware_msgs::DetectedObject generatePredictedObject(const autoware_msgs::DetectedObject &object);

    autoware_msgs::DetectedObject moveConstantVelocity(const autoware_msgs::DetectedObject &object);

    autoware_msgs::DetectedObject moveConstantTurnRateVelocity(const autoware_msgs::DetectedObject &object);

    double generateYawFromQuaternion(const geometry_msgs::Quaternion &quaternion);

    bool isObjectValid(const autoware_msgs::DetectedObject &in_object);

    geometry_msgs::PolygonStamped getPredictedConvexHull(const geometry_msgs::PolygonStamped &in_polygon, const double delta_x, const double delta_y);

public:
    ImmMotionPredict();
    ~ImmMotionPredict();
};

#endif  // NAIVE_MOTION_PREDICT_H
