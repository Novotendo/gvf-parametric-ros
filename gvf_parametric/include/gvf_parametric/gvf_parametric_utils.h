#ifndef GVF_PARAMETRIC_UTILS_H
#define GVF_PARAMETRIC_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <getopt.h>
#include <string>
#include <chrono>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <signal.h>

using namespace std;

typedef struct 
{
  double w;
  double wb;
  double delta_T;
  int s;
  double k_psi;
  double L;
  double beta;
  double kx;
  double ky;
  double heading_rate;
  int perpetual;
} gvf_parametric_con;



typedef struct 
{
  double course;
  double px_dot;
  double py_dot;
} gvf_parametric_st;



vector<double> linspace(double start, double end, int num)
{
  vector<double> linspaced;

  if (num == 0) 
  { 
    return linspaced; 
  }
  else if (num == 1) 
  {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
  {
    linspaced.push_back(start + delta * i);
  }
  
  linspaced.push_back(end);

  return linspaced;
}

#endif