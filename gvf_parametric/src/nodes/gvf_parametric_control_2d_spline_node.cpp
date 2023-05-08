#include <gvf_parametric_control_2d_spline.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gvf_parametric_control_2d_spline_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");

    GVF_PARAMETRIC_CONTROL_2D_SPLINE gvf_parametric_control_2d_spline(nh_, nh_private_);

    ros::spin();

    return 0;
}