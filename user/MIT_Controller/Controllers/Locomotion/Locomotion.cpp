#include "Locomotion.hpp"


template <typename T>
Locomotion<T>::Locomotion(DataReader* data_reader,float _dt) : DataReadCtrl<T>(data_reader, _dt) {
    std::cout <<"initial a data processor!" << std::endl;

  data_processor = DataProcessor();

}

template <typename T>
Locomotion<T>::~Locomotion() {}


template <typename T>
void Locomotion<T>::_update_joint_command() {


  DataCtrl::_des_jpos.setZero();
  DataCtrl::_des_jvel.setZero();
  DataCtrl::_jtorque.setZero();

  DataCtrl::_Kp_joint={10.0, 10.0, 10.0};
  DataCtrl::_Kd_joint={1.0, 1.0, 1.0};

  //DataCtrl::_Kp_joint={20.0, 20.0, 20.0};
  //DataCtrl::_Kd_joint={3.0, 3.0, 3.0};


  // OBTAIN DATA FROM THE JUMP_DATA FILE GENERATED IN MATLAB 
  float* current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration);
  float* tau = current_step + tau_offset;
  
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
  tau_front <<  tau[0], tau[1],  tau[2],  tau[3],  tau[4], tau[5];
  tau_rear << tau[6], tau[7], tau[8] ,  tau[9],  tau[10], tau[11];

// Limitation set so that the arms do not swing too far back and hit the legs 
  if(q_des_front[1] < -M_PI/2.2) {
    q_des_front[1] = -M_PI/2.2;
    qd_des_front[1] = 0.;
    tau_front[1] = 0.;
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



// template <typename T>
// void Locomotion<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) {
//   DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;

//   DataCtrl::_b_Preparation = b_preparation;
//   _update_joint_command();

//   for (int leg = 0; leg < 4; ++leg) {
//     for (int jidx = 0; jidx < 3; ++jidx) {
//       command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];
//       command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx] + 0 * _curr_time;
//       command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];
//       command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];
//       command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];
//       std::cout << "tau" <<  "leg:"  << leg << "link :" << jidx << command[leg].tauFeedForward[jidx] << std::endl;
//       std::cout << "qDes" <<  "leg:"  << leg << "link :" << jidx << command[leg].qDes[jidx] << std::endl;
//       std::cout << "qdDes" <<  "leg:"  << leg << "link :" << jidx << command[leg].qdDes[jidx] << std::endl;

//     }
//   }
// }



template <typename T>
void Locomotion<T>::computeCommand(LegControllerCommand<T>* cmd, LegControllerData<T>* data) {
  // LegControllerCommand<T> * cmd = this->_data->_legController->commands;
  //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = DataCtrl::_des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = DataCtrl::_des_jvel[cheetah::num_leg_joint * leg + jidx];

        cmd[leg].kpJoint(jidx, jidx) =  DataCtrl::_Kp_joint[jidx];
        cmd[leg].kdJoint(jidx, jidx) =  DataCtrl::_Kd_joint[jidx];
       
    }
  }


  // Knee joint non flip barrier
  for(size_t leg(0); leg<4; ++leg){
    if(cmd[leg].qDes[2] < 0.3){
      cmd[leg].qDes[2] = 0.3;
    }
    if(data[leg].q[2] < 0.3){
      T knee_pos = data[leg].q[2]; 
      cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
    }
  }

}


template class Locomotion<double>;
template class Locomotion<float>;
