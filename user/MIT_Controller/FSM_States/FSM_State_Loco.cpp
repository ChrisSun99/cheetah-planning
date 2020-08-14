/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_Loco.h"
#include <Utilities/Utilities_print.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Loco<T>::FSM_State_Loco(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  zero_vec3.setZero();
  f_ff << 0.f, 0.f, -25.f;

  _data_reader = new DataReader(this->_data->_quadruped->_robotType, FSM_StateName::DIRECTCOLLOCATION);
  locomotion_ctrl_ = new Locomotion<T>(_data_reader, this->_data->controlParameters->controller_dt);
  locomotion_ctrl_->SetParameter();
}


template <typename T>
void FSM_State_Loco<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  
  // initial configuration, position
  for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }

  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  locomotion_ctrl_->SetParameter();
  locomotion_ctrl_->_update_joint_command();
}



template<typename T>
bool FSM_State_Loco<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.18)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 9.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Loco<T>::run() {
  locomotion_ctrl_->computeCommand(this->_data->_legController->commands, this->_data->_legController->datas);
}


template <typename T>
bool FSM_State_Loco<T>::_Initialization() { // do away with this?
  static bool test_initialized(false);
  if (!test_initialized) {
    test_initialized = true;
    printf("[Cheetah Test] Test initialization is done\n");
  }
  if (_count < _waiting_count) {
    for (int leg = 0; leg < 4; ++leg) {
      this->_data->_legController->commands[leg].qDes = initial_jpos[leg];
      for (int jidx = 0; jidx < 3; ++jidx) {
        this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
        this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        this->_data->_legController->commands[leg].kpJoint(jidx,jidx) = 20.;
        this->_data->_legController->commands[leg].kdJoint(jidx,jidx) = 2.;
      }
    }
    return true;
  }
  
  return false;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Loco<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;
//  if(locomotionSafe()) {
  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_DIRECTCOLLOCATION:
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;
    
    case K_STAND_UP:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_DIRECTCOLLOCATION << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
//  }
//  else {
//     this->nextStateName = FSM_StateName::RECOVERY_STAND;
//     this->transitionDuration = 0.;
//   }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Loco<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;
    
    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Loco<T>::onExit() {
  // nothing to clean up?
}


template class FSM_State_Loco<float>;
