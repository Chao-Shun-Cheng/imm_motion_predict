#include "imm_motion_predict/imm_motion_predict.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imm_motion_predict");
    ImmMotionPredict node;
    ros::spin();
    return 0;
}
