#ifndef PATHTRACK_H
#define PATHTRACK_H

#include <vector>

namespace track {
struct RoboState {
  RoboState() : v(0) {};

  double x;
  double y;
  double z;

  double yaw;

  double v;
};  // struct RoboState

typedef std::vector<RoboState> StateVec;

class PathTrack {

 public:
  PathTrack() : state_get(false), path_get(false), config_get(false) {}
  ~PathTrack() {}
  void GetState(const RoboState& state);
  void GetPath(const StateVec& vec);
  void GetConfig();
  int ClosestPoint();
  void PubVelocity();
  void FindVelocity();

 private:
  RoboState cur_state;
  bool state_get;
  StateVec path;
  bool path_get;
  bool config_get;

  double linear_v;   // linear velocity of tracked vehicle

  double k;  // forward distance gain
  double l;  // distance between two wheels

};  // class PathTrack

}  // namespace track

#endif
