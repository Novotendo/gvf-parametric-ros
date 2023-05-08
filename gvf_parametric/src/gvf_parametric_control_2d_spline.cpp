#include <gvf_parametric_control_2d_spline.h>

/*
TODO
- Increase speed when initializing the gvf path following, and decrease it when reaching the goal
- Implement 3D path following
*/

GVF_PARAMETRIC_CONTROL_2D_SPLINE::GVF_PARAMETRIC_CONTROL_2D_SPLINE(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{
  nh_private_.param("kx", gvf_parametric_control.kx, gvf_parametric_control.kx);
  nh_private_.param("ky", gvf_parametric_control.ky, gvf_parametric_control.ky);
  nh_private_.param("kpsi", gvf_parametric_control.k_psi, gvf_parametric_control.k_psi);
  nh_private_.param("L", gvf_parametric_control.L, gvf_parametric_control.L);
  nh_private_.param("beta", gvf_parametric_control.beta, gvf_parametric_control.beta);
  nh_private_.param("s", gvf_parametric_control.s, gvf_parametric_control.s);
  nh_private_.param("perpetual", gvf_parametric_control.perpetual, gvf_parametric_control.perpetual);
  nh_private_.param("ground_speed", ground_speed, ground_speed);
  nh_private_.param("lim_angular_speed", lim_angular_speed, lim_angular_speed);
  nh_private_.param("rviz_gvfp_slice_limit", rviz_gvfp_slice_limit, rviz_gvfp_slice_limit);
  nh_private_.param("world_frame", world_frame_, world_frame_);

  odom_sub_ = nh_private_.subscribe("/odometry", 1000, &GVF_PARAMETRIC_CONTROL_2D_SPLINE::pose_cb, this);
  vicon_sub_ = nh_private_.subscribe("/transform", 1000, &GVF_PARAMETRIC_CONTROL_2D_SPLINE::mocap_pose_cb, this);
  trajectory_sub_ = nh_private_.subscribe("/trajectory", 1, &GVF_PARAMETRIC_CONTROL_2D_SPLINE::trajectory_cb, this);
  stop_robot_sub_ = nh_private_.subscribe("/stop_robot", 1, &GVF_PARAMETRIC_CONTROL_2D_SPLINE::stop_robot_cb, this);

  cmd_vel_pub_ = nh_private_.advertise<geometry_msgs::Twist>("/command_velocity", 1);
  rviz_waypoints_vis_pub_ = nh_private_.advertise<visualization_msgs::Marker>("waypoints", 1000);
  rviz_par_spline_vis_pub_ = nh_private_.advertise<visualization_msgs::Marker>("parametric_spline", 1000);
  rviz_proj_par_spline_vis_pub_ = nh_private_.advertise<visualization_msgs::Marker>("projected_parametric_spline", 1000);
  rviz_w_guidence_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("w_guidence", 1000);
  rviz_vector_field_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("gvf", 1000);  
  
  cmdloop_timer_ = nh_private_.createTimer(ros::Duration(0.01), &GVF_PARAMETRIC_CONTROL_2D_SPLINE::cmdloop_cb, this);

  second_loop = false;
  stop = true;
  gvf_parametric_control.heading_rate = 0.0;
  gvf_parametric_control.w = 0.0;
  gvf_parametric_control.delta_T = 0.0;

  signal(SIGINT, GVF_PARAMETRIC_CONTROL_2D_SPLINE::SigintHandler); 
  this_ = this;
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::SigintHandler(int sig)
{
  // For stopping the robot automatically when killing the process (otherwise the robot will continue with the last vel command)
  ROS_WARN("Stopping robot!");
  geometry_msgs::Twist command_end;
  command_end.linear.x = 0.0;
  command_end.angular.z = 0.0;
  this_->cmd_vel_pub_.publish(command_end);
  ros::shutdown();
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::cmdloop_cb(const ros::TimerEvent &event) 
{   
  ros::spinOnce();
  chrono::system_clock::time_point now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - gvf_parametric_t0);
  gvf_parametric_control.delta_T = elapsed.count() * 1e-6; // For ms
  gvf_parametric_t0 = now;

  if(stop)
  {
    waypoints_X.clear();
    waypoints_Y.clear();
    gvf_parametric_control.w = gvf_parametric_control.wb = 0.0;
    command.linear.x = 0.0;
    command.angular.z = 0.0;
    cmd_vel_pub_.publish(command);
    stop = false;
  }
  else if(waypoints_X.size() > 2)
  {
    spline::spline_type type = spline::cspline;
    bool make_monotonic = false;
    bool is_closed_curve = false;
    vector<Eigen::Vector3d> evaluated_spline;

    vector<Eigen::Vector2d> waypoints;
    Eigen::Vector2d point;
    for(int i=0; i < waypoints_X.size(); ++i)
    {
        point.x() = waypoints_X[i];
        point.y() = waypoints_Y[i];
        waypoints.push_back(point);
    }
    rviz_plot_spline_waypoints(waypoints);

    create_parametric_spline(S, waypoints_X, waypoints_Y, type, make_monotonic, is_closed_curve);
    double distance = 0;
    for(int i=0; i < waypoints_X.size()-1; ++i)
    {
        distance = distance + sqrt(pow(waypoints_X[i]-waypoints_X[i+1],2) + pow(waypoints_Y[i]-waypoints_Y[i+1],2));
    }
    double resolution = 0.1;
    int num_points =  distance/resolution;

    evaluate_parametric_spline(evaluated_spline, S, num_points);
    rviz_plot_parametric_spline(evaluated_spline);

    double f1, f2, f1d, f2d, f1dd, f2dd;
    gvf_parametric_2D_spline_info(f1, f2, f1d, f2d, f1dd, f2dd);
    gvf_parametric_control_2D(f1, f2, f1d, f2d, f1dd, f2dd);

    rviz_plot_gvf_parametric_slice_spline(gvf_parametric_control.wb);
    Eigen::Vector3d wb_point(f1, f2, gvf_parametric_control.wb);
    rviz_plot_w_guidence(wb_point);
  }
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::pose_cb(const nav_msgs::OdometryConstPtr &msg)
{
  tf::Quaternion quaternion;
  quaternion.setValue(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 rpy(quaternion);
  double roll, pitch, yaw;
  rpy.getRPY(roll, pitch, yaw);

  current_pose_.x() = msg->pose.pose.position.x;
  current_pose_.y() = msg->pose.pose.position.y;
  current_pose_.z() = yaw;

  gvf_parametric_state.course = current_pose_.z();
  gvf_parametric_state.px_dot = ground_speed * cos(gvf_parametric_state.course); 
  gvf_parametric_state.py_dot = ground_speed * sin(gvf_parametric_state.course); 
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::mocap_pose_cb(const geometry_msgs::TransformStampedConstPtr &msg)
{
  tf::Quaternion quaternion;
  quaternion.setValue(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  tf::Matrix3x3 rpy(quaternion);
  double roll, pitch, yaw;
  rpy.getRPY(roll, pitch, yaw);

  current_pose_.x() = msg->transform.translation.x;
  current_pose_.y() = msg->transform.translation.y;
  current_pose_.z() = yaw;

  gvf_parametric_state.course = current_pose_.z();
  gvf_parametric_state.px_dot = ground_speed * cos(gvf_parametric_state.course); 
  gvf_parametric_state.py_dot = ground_speed * sin(gvf_parametric_state.course); 
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::trajectory_cb(const nav_msgs::PathConstPtr &msg)
{
  if(msg->poses.size() > 2)
  {
    waypoints_X.clear();
    waypoints_Y.clear();
    gvf_parametric_control.w = gvf_parametric_control.wb = 0.0;

    for(auto ptr = msg->poses.begin() ; ptr != msg->poses.end() ; ++ptr)
    {
      //ROS_INFO("Path -> X: [%f] | Y: [%f]", ptr->pose.position.x, ptr->pose.position.y);
      waypoints_X.push_back(ptr->pose.position.x);
      waypoints_Y.push_back(ptr->pose.position.y);
    }
  }
  else
  {
    ROS_WARN("Path contains only %d points. At lest 3 points are required!", msg->poses.size());
  }
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::stop_robot_cb(const std_msgs::Bool& msg)
{
  if(msg.data)
  {
    stop = true;
  }
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::create_w_grid(std::vector<double>& W, double& wmin, double& wmax, std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve)
{
  assert(X.size()==Y.size() && X.size()>2);

  int idx_first=-1, idx_last=-1;
  if(is_closed_curve) 
  {
      // remove last point if it is identical to the first
      if(X[0]==X.back() && Y[0]==Y.back()) 
      {
          X.pop_back();
          Y.pop_back();
      }

      const int num_loops=3;  // number of times we go through the closed loop
      std::vector<double> Xcopy, Ycopy;
      for(int i=0; i<num_loops; i++) 
      {
          Xcopy.insert(Xcopy.end(), X.begin(), X.end());
          Ycopy.insert(Ycopy.end(), Y.begin(), Y.end());
      }
      idx_last  = (int)Xcopy.size()-1;
      idx_first = idx_last - (int)X.size();
      X = Xcopy;
      Y = Ycopy;

      // add first point to the end (so that the curve closes)
      X.push_back(X[0]);
      Y.push_back(Y[0]);
  }

  // setup a "virtual variable" so that we can interpolate x and y
  // coordinates as a function of a virtual variable: (X(w), Y(w))
  W.resize(X.size());
  W[0]=0.0;
  for(size_t i=1; i<W.size(); i++) 
  {
      // proportional to the distance, i.e. we go at a const speed
      W[i] = W[i-1] + sqrt( pow(X[i]-X[i-1],2) + pow(Y[i]-Y[i-1],2) );
  }

  if(idx_first<0 || idx_last<0) 
  {
      wmin = W[0] - 0.0;
      wmax = W.back() + 0.0;
  } 
  else 
  {
      wmin = W[idx_first];
      wmax = W[idx_last];
  }
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::create_parametric_spline(parametric_spline &S, vector<double> &X, vector<double> &Y, spline::spline_type &type, bool &make_monotonic, bool &is_closed_curve)
{
  // setup auxiliary "time grid"
  S.wmin = 0.0, S.wmax = 0.0;
  vector<double> W;
  create_w_grid(W,S.wmin,S.wmax,X,Y,is_closed_curve);

  // define a spline for each coordinate x, y
  spline sx, sy;
  sx.set_points(W,X,type);
  sy.set_points(W,Y,type);
  if(make_monotonic) 
  {
    // adjusts spline coeffs to be piecewise monotonic where possible
    sx.make_monotonic();
    sy.make_monotonic();
  }
  S.sx = sx;
  S.sy = sy;
  S.wmin = W.front();
  S.wmax = W.back();
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::evaluate_parametric_spline(vector<Eigen::Vector3d> &evaluated_spline, parametric_spline &S, int num_grid_points)
{
  Eigen::Vector3d point;

  for(int i=0; i<num_grid_points; i++) 
  {
    double w = S.wmin + (double)i*(S.wmax-S.wmin)/(num_grid_points-1);
    point.x() = S.sx(w);
    point.y() = S.sy(w);
    point.z() = w;
    evaluated_spline.push_back(point);
  }
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::gvf_parametric_2D_spline_info(double &f1, double &f2, double &f1d, double &f2d, double &f1dd, double &f2dd)
{
  double w = gvf_parametric_control.w;
  double wb = gvf_parametric_control.wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  spline_coeff spline_coeff_x = S.sx.get_spline_coeff(wb);
  spline_coeff spline_coeff_y = S.sy.get_spline_coeff(wb);
  double ax = spline_coeff_x.a;
  double bx = spline_coeff_x.b;
  double cx = spline_coeff_x.c;
  double dx = spline_coeff_x.d;
  double inix = spline_coeff_x.x_ini;
  double ay = spline_coeff_y.a;
  double by = spline_coeff_y.b;
  double cy = spline_coeff_y.c;
  double dy = spline_coeff_y.d;
  double iniy = spline_coeff_y.x_ini; 

  double hx = wb - inix;
  double hy = wb - iniy;

  f1 = ax + bx*hx + cx*pow(hx,2) + dx*pow(hx,3);
  f2 = ay + by*hy + cy*pow(hy,2) + dy*pow(hy,3);

  f1d = bx + 2*cx*hx + 3*dx*pow(hx,2);
  f2d = by + 2*cy*hy + 3*dy*pow(hy,2);

  f1dd = 2*cx + 6*dx*hx;
  f2dd = 2*cy + 6*dy*hy;
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::gvf_parametric_control_2D(double &f1, double &f2, double &f1d, double &f2d, double &f1dd, double &f2dd)
{
  if(second_loop) 
  { // Two loop times samples needed
    if(gvf_parametric_control.wb >= S.wmax)
    {
      if(gvf_parametric_control.perpetual == 1)
      { // If perpetual trajectory, reset w
        gvf_parametric_control.w = 0;
      }
      else
      { // If only once, then stop the robot
        command.linear.x = 0.0;
        command.angular.z = 0.0;
        waypoints_X.clear();
        waypoints_Y.clear();
      }
    }
    else
    {
      double L = gvf_parametric_control.L;
      double beta = gvf_parametric_control.beta * gvf_parametric_control.s;

      Eigen::Vector3d X;
      Eigen::Matrix3d J;

      double x = current_pose_.x();
      double y = current_pose_.y();

      double phi1 = L * (x - f1);
      double phi2 = L * (y - f2);

      // Chi
      double kx = gvf_parametric_control.kx;
      double ky = gvf_parametric_control.ky;
      X(0) = L * beta * f1d - kx * phi1;
      X(1) = L * beta * f2d - ky * phi2;
      X(2) = L + beta * (kx * phi1 * f1d + ky * phi2 * f2d);
      X *= L;

      // Jacobian
      J.setZero();
      J(0, 0) = -kx * L;
      J(1, 1) = -ky * L;
      J(2, 0) = (beta * L) * (beta * f1dd + kx * f1d);
      J(2, 1) = (beta * L) * (beta * f2dd + ky * f2d);
      J(2, 2) = beta * beta * (kx * (phi1 * f1dd - L * f1d * f1d) + ky * (phi2 * f2dd - L * f2d * f2d));
      J *= L;

      // Guidance algorithm
      double w_dot = (ground_speed * X(2)) / sqrtf(X(0) * X(0) + X(1) * X(1));
      Eigen::Vector3d xi_dot;
      double course = gvf_parametric_state.course;
      xi_dot << gvf_parametric_state.px_dot, gvf_parametric_state.py_dot, w_dot;

      Eigen::Matrix3d G;
      Eigen::Matrix3d Gp;
      Eigen::Matrix<double, 2, 3> Fp;
      Eigen::Vector2d h;
      Eigen::Matrix<double, 1, 2> ht;

      G << 1, 0, 0,
      0, 1, 0,
      0, 0, 0;
      Fp << 0, -1, 0,
      1,  0, 0;
      Gp << 0, -1, 0,
      1,  0, 0,
      0,  0, 0;

      h << cos(course), sin(course);
      ht = h.transpose();

      Eigen::Matrix<double, 1, 3> Xt = X.transpose();
      Eigen::Vector3d Xh = X / X.norm();
      Eigen::Matrix<double, 1, 3> Xht = Xh.transpose();
      Eigen::Matrix3d I;
      I.setIdentity();

      double aux = ht * Fp * X;
      double heading_rate = -1 / (Xt * G * X) * Xt * Gp * (I - Xh * Xht) * J * xi_dot - (gvf_parametric_control.k_psi * aux / sqrtf(Xt * G * X));

      // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameter w will get "closer" to
      // the vehicle. So it is not only okei but advisable to update it.
      gvf_parametric_control.w += w_dot * gvf_parametric_control.delta_T * 1e-3;
      gvf_parametric_control.heading_rate = heading_rate;
      command.linear.x = ground_speed;

      // Saturation for non-realistic robot velocities
      if(gvf_parametric_control.heading_rate >= lim_angular_speed)
      {
        command.angular.z = lim_angular_speed;
      }
      else if(gvf_parametric_control.heading_rate <= -lim_angular_speed)
      {
        command.angular.z = -lim_angular_speed;
      }
      else
      {
        command.angular.z = gvf_parametric_control.heading_rate;
      }
    }
    cmd_vel_pub_.publish(command);
  }
  second_loop = true;
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::rviz_plot_spline_waypoints(vector<Eigen::Vector2d> &points)
{
  visualization_msgs::Marker marker;
  geometry_msgs::Point position;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

	for(auto ptr = points.begin() ; ptr != points.end() ; ++ptr)
  {
		position.x = ptr -> x();
		position.y = ptr -> y();
		marker.points.push_back(position);
	}
  rviz_waypoints_vis_pub_.publish(marker);
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::rviz_plot_parametric_spline(vector<Eigen::Vector3d> &points)
{
  // For 3D representation
  visualization_msgs::Marker marker;
  geometry_msgs::Point position;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "3d_parametric_spline";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.08;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

	for(auto ptr = points.begin() ; ptr != points.end() ; ++ptr)
  {
		position.x = ptr -> x();
		position.y = ptr -> y();
    position.z = ptr -> z();
		marker.points.push_back(position);
	}
  rviz_par_spline_vis_pub_.publish(marker);

  // For 2D projection
  marker.points.clear();
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "2d_projected_parametric_spline";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.08;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

	for(auto ptr = points.begin() ; ptr != points.end() ; ++ptr)
  {
		position.x = ptr -> x();
		position.y = ptr -> y();
    position.z = 0.0;
		marker.points.push_back(position);
	}
  rviz_proj_par_spline_vis_pub_.publish(marker);
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::rviz_plot_w_guidence(Eigen::Vector3d &point)
{
  visualization_msgs::MarkerArray w_g;
  visualization_msgs::Marker marker;
  geometry_msgs::Point position;

  // For 3D representation
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "w_guidence";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  w_g.markers.push_back(marker);

  // For 2D projection
  marker.points.clear();
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "w_guidence_projected";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = 0.4; 
  marker.scale.z = 0.001;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = 0.0;
  w_g.markers.push_back(marker);

  rviz_w_guidence_pub_.publish(w_g);
}



void GVF_PARAMETRIC_CONTROL_2D_SPLINE::rviz_plot_gvf_parametric_slice_spline(double &wb)
{
  visualization_msgs::MarkerArray vector_field;
  int samples = 4 * rviz_gvfp_slice_limit;
  vector<double> samples_x = linspace(-rviz_gvfp_slice_limit, rviz_gvfp_slice_limit, samples);
  vector<double> samples_y = linspace(-rviz_gvfp_slice_limit, rviz_gvfp_slice_limit, samples);
  int id_marker = 0;

  for (auto itx = samples_x.begin() ; itx != samples_x.end() ; ++itx) 
  {
    for (auto ity = samples_y.begin() ; ity != samples_y.end() ; ++ity) 
    {
      spline_coeff spline_coeff_x = S.sx.get_spline_coeff(wb);
      spline_coeff spline_coeff_y = S.sy.get_spline_coeff(wb);
      double ax = spline_coeff_x.a;
      double bx = spline_coeff_x.b;
      double cx = spline_coeff_x.c;
      double dx = spline_coeff_x.d;
      double inix = spline_coeff_x.x_ini;
      double ay = spline_coeff_y.a;
      double by = spline_coeff_y.b;
      double cy = spline_coeff_y.c;
      double dy = spline_coeff_y.d;
      double iniy = spline_coeff_y.x_ini; 

      double hx = wb - inix;
      double hy = wb - iniy;

      double f1 = ax + bx*hx + cx*pow(hx,2) + dx*pow(hx,3);
      double f2 = ay + by*hy + cy*pow(hy,2) + dy*pow(hy,3);

      double f1d = bx + 2*cx*hx + 3*dx*pow(hx,2);
      double f2d = by + 2*cy*hy + 3*dy*pow(hy,2);

      double L = gvf_parametric_control.L;
      double beta = gvf_parametric_control.beta * gvf_parametric_control.s;

      Eigen::Vector3f X;

      // Error signals phi_x and phi_y
      double x = *itx;
      double y = *ity;
      double z = wb;

      double phi1 = L * (x - f1);
      double phi2 = L * (y - f2);

      // Chi
      X(0) = L * beta * f1d - gvf_parametric_control.kx * phi1;
      X(1) = L * beta * f2d - gvf_parametric_control.ky * phi2;
      X(2) = L + beta * (gvf_parametric_control.kx * phi1 * f1d + gvf_parametric_control.ky * phi2 * f2d);
      X *= L;

      double pdx_dot = X(0);
      double pdy_dot = X(1);
      double pdz_dot = X(2);

      geometry_msgs::Point xyz_1, xyz_2;
      visualization_msgs::Marker edge;

      edge.header.frame_id = world_frame_;
      edge.header.stamp = ros::Time();
      edge.ns = "gvf_parametric_gradient";
      edge.action = visualization_msgs::Marker::ADD;
      id_marker++;
      edge.id = id_marker;
      edge.type = visualization_msgs::Marker::ARROW;
      edge.scale.x = 0.03;
      edge.scale.y = 0.15;
      edge.scale.z = 0.1;
      edge.color.a = 1.0;
      edge.color.r = 0.0;
      edge.color.g = 1.0;
      edge.color.b = 0.0;
      edge.pose.orientation.w = 1.0;

      xyz_1.x = x;
      xyz_1.y = y;
      xyz_1.z = z;

      double magnitude = sqrt((pow(pdx_dot,2)) + pow(pdy_dot,2) + (pow(pdz_dot,2)));
      double scale = 0.4;

      xyz_2.x = scale * pdx_dot / magnitude + x;
      xyz_2.y = scale * pdy_dot / magnitude + y;
      xyz_2.z = scale * pdz_dot / magnitude + z;

      edge.points.push_back(xyz_1);
      edge.points.push_back(xyz_2);

      vector_field.markers.push_back(edge);
    }
  }
  rviz_vector_field_pub_.publish(vector_field);
}