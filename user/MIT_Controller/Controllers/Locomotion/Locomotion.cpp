#include "Locomotion.hpp"


template <typename T>
Locomotion<T>::Locomotion(DataReader* data_reader,float _dt) : DataReadCtrl<T>(data_reader, _dt) {
  std::cout <<"initial a data processor!" << std::endl;
  // data_processor = DataProcessor();

  // Make sure call dataporcessor() only once; 
  // static std::once_flag flag;
  // std::call_once(flag, []{new DataProcessor();});
}

template <typename T>
Locomotion<T>::~Locomotion() {}

template <typename T>
void Locomotion<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) {
  DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;
  DataCtrl::_b_Preparation = b_preparation;

  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];
      command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx] + 0 * _curr_time;
      command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];
      command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];
      command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];
      // std::cout << "tau" <<  "leg:"  << leg << "   " << "link :" << jidx << "   " << command[leg].tauFeedForward[jidx] << std::endl;
      // std::cout << "qDes" <<  "leg:"  << leg << "   " << "link :" << jidx << "   " << command[leg].qDes[jidx] << std::endl;
      // std::cout << "qdDes" <<  "leg:"  << leg <<"   " << "link :" << jidx << "   " <<  command[leg].qdDes[jidx] << std::endl;
    }
  }
}

template <typename T>
void Locomotion<T>::_update_joint_command() {

  int pre_mode_duration(10);
  int leg_clearance_iteration_front(240) ; 
  //int leg_clearance_iteration_front(180) ; 
  int leg_clearance_iteration(600);
  int leg_ramp_iteration(610);
  int tuck_iteration(610);
  int ramp_end_iteration(700);

  float tau_mult;

  DataCtrl::_des_jpos.setZero();
  DataCtrl::_des_jvel.setZero();
  DataCtrl::_jtorque.setZero();

  DataCtrl::_Kp_joint={10.0, 10.0, 10.0};
  DataCtrl::_Kd_joint={1.0, 1.0, 1.0};

  // PRE JUMP PREPATATION - CROUCH (FOLLOWS PREMODE DURATION TIMESTEPS) 
  if ( (DataCtrl::pre_mode_count <  pre_mode_duration) || DataCtrl::_b_Preparation) {  
    // move to the initial configuration to prepare for
    // FrontJumping
    if (DataCtrl::pre_mode_count == 0) {
      printf("plan_timesteps: %d \n", DataCtrl::_data_reader->plan_timesteps);
    }
    // printf("pre_mode_count: %d \n", pre_mode_count);
    
    DataCtrl::pre_mode_count += DataCtrl::_key_pt_step;
    DataCtrl::current_iteration = 0;
    tau_mult = 0;
    //DataCtrl::_Kp_joint={20.0, 20.0, 20.0};
    //DataCtrl::_Kd_joint={2.0, 2.0, 2.0};

  } else {
    tau_mult = 1.2;
    // tau_mult = 1.;
  }

  //DataCtrl::_Kp_joint={20.0, 20.0, 20.0};
  //DataCtrl::_Kd_joint={3.0, 3.0, 3.0};
 // OBTAIN TIMSTEP DATA FROM THE DATA FILE 
  if (DataCtrl::current_iteration > DataCtrl::_data_reader->plan_timesteps - 1) {
    DataCtrl::current_iteration = DataCtrl::_data_reader->plan_timesteps - 1;
  }

  // OBTAIN DATA FROM THE JUMP_DATA FILE GENERATED IN MATLAB 
  float* current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration);
  

  // std::cout << "current iteration" << DataCtrl::current_iteration << std::endl;
  // float* tau = current_step + tau_offset;
  // std::cout << "dataReader" << std::endl;
  // for (int cur = 0; cur < 60; cur++) {
  //   std::cout  << current_step[cur] << std::endl;
  // }

  // INITIALIZE JOINT PARAMETERS AND TORQUES 
  Vec6<float> q_des_front;
  Vec6<float> q_des_rear;
  Vec6<float> qd_des_front;
  Vec6<float> qd_des_rear;
  Vec6<float> tau_front;
  Vec6<float> tau_rear;

  // SETTING THE JOINT POSITIONS AND VELOCITIES AND FEEDING FORWARD THE JOINT TORQUES obtained from the data file 

  //  x, y, z, raw, pitch, yaw, front ad, front hip, front knee, rear ad, rear hip, rear knee
  q_des_front << current_step[6], current_step[7], current_step[8], current_step[9], current_step[10], current_step[11];
  q_des_rear << current_step[12], current_step[13], current_step[14], current_step[15], current_step[16], current_step[17];
  qd_des_front << current_step[24], current_step[25], current_step[26], current_step[27], current_step[28], current_step[29];
  qd_des_rear << current_step[30], current_step[31], current_step[32], current_step[33], current_step[34], current_step[35];
  tau_front << tau_mult * current_step[36] /2.0, tau_mult * current_step[37]/2.0,  tau_mult * current_step[38]/2.0, tau_mult *  current_step[39]/2.0, tau_mult *  current_step[40]/2.0,  tau_mult * current_step[41]/2.0;
  tau_rear <<  tau_mult * current_step[42]/2.0, tau_mult * current_step[43]/2.0,  tau_mult *  current_step[44]/2.0, tau_mult * current_step[45]/2.0,  tau_mult * current_step[46]/2.0, tau_mult * current_step[47]/2.0;

  // Limitation set so that the arms do not swing too far back and hit the legs 
   if(q_des_front[1] < -M_PI/2.2) {
    q_des_front[1] = -M_PI/2.2;
    qd_des_front[1] = 0.;
    tau_front[1] = 0.;
  }

  std::cout << "PRINTING CURRENT_STEP!!!" << std::endl;
  std::cout << "PRINT CURR ITER: " << DataCtrl::current_iteration << std::endl;
  // for (int index = 0; index < 60; index++) {
  //   std::cout << current_step[index] << std::endl;
  // }

  std::cout << "qDes:  " << std::endl;
  for (int i = 0; i < 6; i++ ) {
    std::cout << q_des_front[i]<<  " ";
    std::cout << q_des_rear[i] <<"  ";
  }

   std::cout << "qdDes:  " << std::endl;
  for (int i = 0; i < 6; i++ ) {
    std::cout << qd_des_front[i]<<  " ";
    std::cout << qd_des_rear[i]<<  " ";
  }
  

  std::cout << "tau:  " << std::endl;
  for (int i = 0; i < 6; i++ ) {
    std::cout << tau_front[i] <<  " ";
    std::cout << tau_rear[i] <<  " ";
  }
  
  

  float s(0.);

  for (int i = 0; i < 12; i += 3) {
    DataCtrl::_des_jpos[i] = 0.0;
    DataCtrl::_des_jvel[i] = 0.0;
    DataCtrl::_jtorque[i] = 0.0;
  }
  DataCtrl::_des_jpos[0] = s * (-0.2);
  DataCtrl::_des_jpos[3] = s * (0.2);
  DataCtrl::_des_jpos[6] = s * (-0.2);
  DataCtrl::_des_jpos[9] = s * (0.2);

  //Front Left Ad
  DataCtrl::_des_jpos[0] = q_des_front[0];
  DataCtrl::_des_jvel[0] = qd_des_front[0];
  DataCtrl::_jtorque[0] = tau_front[0];


  //Front Right Ad 

  DataCtrl::_des_jpos[3] = q_des_front[3];
  DataCtrl::_des_jvel[3] = qd_des_front[3];
  DataCtrl::_jtorque[3] = tau_front[3];
  

  // Front Left Hip

    DataCtrl::_des_jpos[1] = q_des_front[1];
    DataCtrl::_des_jvel[1] = qd_des_front[1];
    DataCtrl::_jtorque[1] = tau_front[1];
  

  // Front Right Hip

    DataCtrl::_des_jpos[4] = q_des_front[4];
    DataCtrl::_des_jvel[4] = qd_des_front[4];
    DataCtrl::_jtorque[4] = tau_front[4];
  


  // Front Left Knee
    DataCtrl::_des_jpos[2] = q_des_front[2];
    DataCtrl::_des_jvel[2] = qd_des_front[2];
    DataCtrl::_jtorque[2] = tau_front[2];
  

  //Front Right Knee
    DataCtrl::_des_jpos[5] = q_des_front[5];
    DataCtrl::_des_jvel[5] = qd_des_front[5];
    DataCtrl::_jtorque[5] = tau_front[5];
  

  //Hind Left Ad 

    DataCtrl::_des_jpos[6] = q_des_rear[0];
    DataCtrl::_des_jvel[6] = qd_des_rear[0];
    DataCtrl::_jtorque[6] = tau_rear[0];
  


  // Hind Left Hip
    DataCtrl::_des_jpos[7] = q_des_rear[1];
    DataCtrl::_des_jvel[7] = qd_des_rear[1];
    DataCtrl::_jtorque[7] = tau_rear[1];
  

  // Hind Left Knee
    DataCtrl::_des_jpos[8] = q_des_rear[2];
    DataCtrl::_des_jvel[8] = qd_des_rear[2];
    DataCtrl::_jtorque[8] = tau_rear[2];
  

   //Hind Right Ad 
    DataCtrl::_des_jpos[9] = q_des_rear[3];
    DataCtrl::_des_jvel[9] = qd_des_rear[3];
    DataCtrl::_jtorque[9] = tau_rear[3];
  

  // Hind Right Hip
    DataCtrl::_des_jpos[10] = q_des_rear[4];
    DataCtrl::_des_jvel[10] = qd_des_rear[4];
    DataCtrl::_jtorque[10] = tau_rear[4];
  

  // Hind Right Knee
    DataCtrl::_des_jpos[11] = q_des_rear[5];
    DataCtrl::_des_jvel[11] = qd_des_rear[5];
    DataCtrl::_jtorque[11] = tau_rear[5];

  // Update rate 0.5kHz
  DataCtrl::current_iteration += DataCtrl::_key_pt_step;
}


template class Locomotion<double>;
template class Locomotion<float>;
