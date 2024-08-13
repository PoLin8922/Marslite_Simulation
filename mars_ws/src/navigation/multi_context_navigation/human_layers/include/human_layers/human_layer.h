/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/
 
#ifndef HUMAN_LAYERS_H
#define HUMAN_LAYERS_H
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <human_msgs/TrackedHumans.h>
#include <human_msgs/StateArray.h>
#include <human_msgs/TrackedSegmentType.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <math.h>

#define PI 3.14159265359

namespace human_layers
{
class HumanLayer : public costmap_2d::Layer
{
public:
  HumanLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  virtual void updateBoundsFromHumans(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized()
  {
    return false;
  }

protected:
  struct HumanPoseVel{
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
  };

  void humansCB(const human_msgs::TrackedHumans& humans);

  void statesCB(const human_msgs::StateArray& states);

  double Guassian1D(double x, double x0, double A, double varx){
    double dx = x-x0;
    return A*exp(-pow(dx,2.0)/(2.0*varx));
  }

  double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta), Y = d*sin(theta);
    return (A/2)/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta-skew_ang), Y = d*sin(theta-skew_ang);
    return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double getRadius(double cutoff, double A, double var)
  {
    return sqrt(-2 * var * log(cutoff / A));
  }

  /******** Asymmetrical Gaussian ********/
  // normalize to [-π, π]
  double normalize(double angle) {
      while (angle > PI) angle -= 2 * PI;
      while (angle < -PI) angle += 2 * PI;
      return angle;
  }

  double Asymmetrical_Gaussian(double x, double y, double x0, double y0, double vx, double vy, double var, double r_ratio, double A) {
      // define parameter
      double v = sqrt(vx * vx + vy * vy);
      double theta = atan2(vy, vx);
      double sigmaHead = fmax(1.2 * v, var);
      // double sigmaHead = fmax(1.2 * v, 1.0);
      // double sigmaRear = var / 7.0;
      // double sigmaLarge = var / 5.0;
      double sigmaRear = var * 0.7;
      double sigmaLarge = var ;
      double sigmaSmall = var * r_ratio;

      // compute αmain, αside
      double alphaMain = normalize(atan2(y - y0, x - x0) - theta + PI / 2);
      double alphaSide1 = normalize(alphaMain - PI / 2); // right hand principle
      // double alphaSide2 = normalize(alphaMain + PI / 2); // lefy hand principle
 
      // determin sigmaMain, sigmaSide
      double sigmaMain, sigmaSide;
      if (alphaMain > 0) {
          sigmaMain = sigmaHead;
      } else {
          sigmaMain = sigmaRear;
      }

      // if (alphaSide1 > 0 || alphaSide2 > 0) {
      if (alphaSide1 > 0) {
          sigmaSide = sigmaLarge;
      } else {
          sigmaSide = sigmaSmall;
      }

      // determin G,、Gb, Gc
      double cosTheta = cos(theta);
      double sinTheta = sin(theta);
      double Ga = (cosTheta * cosTheta) / (2 * sigmaMain * sigmaMain) + (sinTheta * sinTheta) / (2 * sigmaSide * sigmaSide);
      double Gb = sin(2 * theta) / (4 * sigmaMain * sigmaMain) - sin(2 * theta) / (4 * sigmaSide * sigmaSide);
      double Gc = (sinTheta * sinTheta) / (2 * sigmaMain * sigmaMain) + (cosTheta * cosTheta) / (2 * sigmaSide * sigmaSide);

      // compute social cost
      double result = A*exp(-1.0 * (Ga * (x - x0) * (x - x0) + 2 * Gb * (x - x0) * (y - y0) + Gc * (y - y0) * (y - y0)));

      return result;
  }         
  /******** Asymmetrical Gaussian ********/

  ros::Subscriber humans_sub_, humans_states_sub_;
  human_msgs::TrackedHumans humans_;
  human_msgs::StateArray states_;
  std::vector<HumanPoseVel> transformed_humans_;
  boost::recursive_mutex lock_;
  bool first_time_, reset;
  ros::Time last_time;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double radius_, amplitude_, r_ratio_, covar_, cutoff_;
  // double v_cutoff_, v_amplitude_, v_covar_;

  ros::Timer timeout_timer_;
  ros::Time last_update_time_;
  void timeoutCheck(const ros::TimerEvent& event);

};
}  // namespace human_layers

#endif  // HUMAN_LAYERS_H