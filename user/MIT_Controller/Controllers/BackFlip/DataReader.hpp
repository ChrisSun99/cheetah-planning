#ifndef BACKFLIP_DATA_READER_H
#define BACKFLIP_DATA_READER_H
#include <cppTypes.h>
#include <FSM_States/FSM_State.h>

// enum plan_offsets {
//   q0_offset = 0,     // x, z, yaw, front hip, front knee, rear hip, rear knee
//   qd0_offset = 7,    // x, z, yaw, front hip, front knee, rear hip, rear knee
//   tau_offset = 14,   // front hip, front knee, rear hip, rear knee
//   force_offset = 18  // front x, front z, rear x, rear z
// };
enum plan_offsets {
  q0_offset = 0,     // x, y, z, raw, pitch, yaw, front left ad, front right ad, front left hip, front right hip, front left knee, front right knee, rear left ad, rear right ad, rear left hip, rear right hip, rear left knee, rear right knee
  qd0_offset = 18,    // x, y, z, raw, pitch, yaw, front left ad, front right ad, front left hip, front right hip, front left knee, front right knee, rear left ad, rear right ad, rear left hip, rear right hip, rear left knee, rear right knee
  tau_offset = 36,   // front left ad, front right ad, front left hip, front right hip, front left knee, front right knee, rear left ad, rear right ad, rear left hip, rear right hip, rear left knee, rear right knee
  force_offset = 48  // front left x, front right x, front left y, front right y, front left z, front right z, rear left x, rear right x, rear left y, rear right y, rear left z, rear right z
};

typedef Eigen::Matrix<float, 7, 1> Vector7f;

class DataReader {
 public:
  static const int plan_cols = 60;

  DataReader(const RobotType &, FSM_StateName stateNameIn);
  void load_control_plan(const char *filename);
  void unload_control_plan();
  float *get_initial_configuration();
  float *get_plan_at_time(int timestep);
  int plan_timesteps = -1;

 private:
  RobotType _type;
  float *plan_buffer;
  bool plan_loaded = false;
};

#endif  // BACKFLIP_DATA_READER_H
