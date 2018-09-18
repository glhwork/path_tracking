#ifndef PATHTRACK_H
#define PATHTRACK_H

#include <vector>

namespace track {
struct RoboState {
  RoboState() : v(0) {};

  double x;
  double y;
  double z;

  double v;
};  // struct RoboState

typedef std::vector<RoboState> StateVec;

class PathTrack {
 public:
  PathTrack();
  ~PathTrack();
  void GetStateCallback();
  void GetPathCallback();
  void PubVelocity();
  void FindV();
private:
 RoboState cur_state;
 StateVec path;

};  // class PathTrack

}  // namespace track












#endif
