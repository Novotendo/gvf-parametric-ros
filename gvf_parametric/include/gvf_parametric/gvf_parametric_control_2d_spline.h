#ifndef GVF_PARAMETRIC_CONTROL_2D_SPLINE_H
#define GVF_PARAMETRIC_CONTROL_2D_SPLINE_H

#include <gvf_parametric_utils.h>
#include <spline.h>

using namespace tk;

class GVF_PARAMETRIC_CONTROL_2D_SPLINE
{

    public:
        GVF_PARAMETRIC_CONTROL_2D_SPLINE(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        virtual ~GVF_PARAMETRIC_CONTROL_2D_SPLINE() {};

        // Signal handler
        static void SigintHandler(int sig);

        // Callbacks
        void cmdloop_cb(const ros::TimerEvent &event);
        void pose_cb(const nav_msgs::OdometryConstPtr &msg);
        void mocap_pose_cb(const geometry_msgs::TransformStampedConstPtr &msg);
        void trajectory_cb(const nav_msgs::PathConstPtr &msg);
        void stop_robot_cb(const std_msgs::Bool& msg);

        // Parametric Splines
        void create_w_grid(std::vector<double>& W, double& wmin, double& wmax, std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve);
        void create_parametric_spline(parametric_spline &S, vector<double> &X, vector<double> &Y, spline::spline_type &type, bool &make_monotonic, bool &is_closed_curve);
        void evaluate_parametric_spline(vector<Eigen::Vector3d> &evaluated_spline, parametric_spline &S, int num_grid_points);

        // GVF Parametric
        void gvf_parametric_2D_spline_info(double &f1, double &f2, double &f1d, double &f2d, double &f1dd, double &f2dd);
        void gvf_parametric_control_2D(double &f1, double &f2, double &f1d, double &f2d, double &f1dd, double &f2dd);

        // Rviz
        void rviz_plot_spline_waypoints(vector<Eigen::Vector2d> &points);
        void rviz_plot_parametric_spline(vector<Eigen::Vector3d> &points);
        void rviz_plot_w_guidence(Eigen::Vector3d &point);
        void rviz_plot_gvf_parametric_slice_spline(double &wb);   


    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Timer cmdloop_timer_;

        static inline const GVF_PARAMETRIC_CONTROL_2D_SPLINE* this_;

        ros::Subscriber odom_sub_, vicon_sub_, trajectory_sub_, stop_robot_sub_;
        ros::Publisher cmd_vel_pub_, rviz_waypoints_vis_pub_, rviz_par_spline_vis_pub_, rviz_proj_par_spline_vis_pub_, 
                        rviz_w_guidence_pub_, rviz_vector_field_pub_;

        Eigen::Vector3d current_pose_;

        gvf_parametric_st gvf_parametric_state;
        gvf_parametric_con gvf_parametric_control;

        chrono::system_clock::time_point gvf_parametric_t0;

        parametric_spline S;

        bool second_loop, stop;
        double ground_speed, lim_angular_speed, rviz_gvfp_slice_limit;

        vector<double> waypoints_X, waypoints_Y;

        geometry_msgs::Twist command;

        std::string world_frame_;
};

#endif