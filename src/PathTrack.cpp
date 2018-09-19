
#include <Eigen/Dense>

#include "path_tracking/PathTrack.h"


void track::PathTrack::GetState(const RoboState& state) {
  
  cur_state = state;
  state_get = true;

}

void track::PathTrack::GetPath(const StateVec& vec) {
  
  path = vec;
  path_get = true;

}

void track::PathTrack::GetConfig() {
  /* read a file to get the configuration of the tracked vehicle */
  config_get = true;
}

void track::PathTrack::FindVelocity() {
  
  if (state_get && path_get && config_get) {

  	int index = ClosestPoint();
  	double ld = k * cur_state.v;
  	double min_diff = 100000.0;
  	int ind;

  	// to find a point on the path whose distance from current 
  	// position is the closest to the forward distance  	
  	for (size_t i = index; i < path.size(); i++) {
  	  double dis = sqrt(pow((cur_state.x - path[i].x) ,2) +
  			            pow((cur_state.y - path[i].y) ,2) +
  			            pow((cur_state.z - path[i].z) ,2));
  	  double diff = abs(dis - ld);
  	  if (diff < min_diff) {
  	  	min_diff = diff;
  	  	ind = i;
  	  }
  	}

  	Eigen::Vector3d forward_vec(path[ind].x - cur_state.x,
  							    path[ind].y - cur_state.y,
  							    path[ind].z - cur_state.z);
    Eigen::Vector3d x_axis(1, 0, 0);
    
    // alpha : angle between heading and forward distance vector
    // beta  : angle between x-axis and forward distance vector
    double alpha, beta;  
    beta = acos(x_axis.dot(forward_vec) / forward_vec.norm());
    alpha = cur_state.yaw - beta;
    double radius = ld / (2 * sin(alpha));

    double left_v, right_v;
    left_v = linear_v * (1 + l / (2 * radius));
    right_v = linear_v * (1 - l / (2 * radius));

  }

}

void track::PathTrack::PubVelocity() {
  
}

int track::PathTrack::ClosestPoint() {

  double min_dis = 100000.0;
  int index;
  for (size_t i = 0; i < path.size(); i++) {
  	double dis = sqrt(pow((cur_state.x - path[i].x) ,2) +
  			          pow((cur_state.y - path[i].y) ,2) +
  			          pow((cur_state.z - path[i].z) ,2));
  	if (dis < min_dis) {
  	  min_dis = dis;
  	  index = i;
  	}
  }

  return index;
}