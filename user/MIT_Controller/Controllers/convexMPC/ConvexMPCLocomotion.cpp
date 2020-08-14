// #include <iostream>
// #include <Utilities/Timer.h>
// #include <Utilities/Utilities_print.h>

// #include "ConvexMPCLocomotion.h"
// #include "convexMPC_interface.h"
// #include "../../../../common/FootstepPlanner/GraphSearch.h"

// #include "Gait.h"
// #include <cmath>
// #include <ctime> 
// #include <ratio>
// #include <chrono> 
// #include <fstream> 
// #include <json/writer.h>
// #include <json/value.h>
// #include <json/reader.h>
// #include <cppTypes.h>
// #include <boost/format.hpp> 
// #include <map>
// #include <iterator>
// #include <libInterpolate/Interpolate.hpp>
// #include "rapidjson/document.h"
// #include "rapidjson/writer.h"
// #include "rapidjson/stringbuffer.h"
// #include <typeinfo>

// ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
//   iterationsBetweenMPC(_iterations_between_mpc),
//   horizonLength(10),
//   dt(_dt),
//   trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
//   bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
//   //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
//   pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
//   jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
//   //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
//   //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
//   galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
//   standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
//   //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
//   trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
//   walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
//   walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
//   pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
//   random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
//   random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
// {
//   _parameters = parameters;
//   dtMPC = dt * iterationsBetweenMPC;
//   default_iterations_between_mpc = iterationsBetweenMPC;
//   printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
//   setup_problem(dtMPC, horizonLength, 0.4, 120);
//   //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
//   rpy_comp[0] = 0;
//   rpy_comp[1] = 0;
//   rpy_comp[2] = 0;
//   rpy_int[0] = 0;
//   rpy_int[1] = 0;
//   rpy_int[2] = 0;

//   for(int i = 0; i < 4; i++)
//     firstSwing[i] = true;

//   initSparseMPC();

//    pBody_des.setZero();
//    vBody_des.setZero();
//    aBody_des.setZero();


//   std::ifstream file_in("/home/chris/Desktop/latest/Cheetah-Software/user/MIT_Controller/output.json", std::ifstream::binary);
//   if (!file_in.is_open()){
//        std::cout << "Error opening file"<< std::endl;
//        exit (1);
//   }
 
//   bool parseSuccessful = reader.parse(file_in, opt);
//   if (!parseSuccessful) {
//     std::cout << "ERROR DURING PARSING JSON FILE.";
//   }
//   tt = std::to_string(t);
  

//   using namespace std::chrono;
//   time1 = high_resolution_clock::now();
//   duration = high_resolution_clock::duration::zero();
//   prev_duration = high_resolution_clock::duration::zero();
//   curr_duration = high_resolution_clock::duration::zero();

//   // // Time Interpolation
//   // _1D::LinearInterpolator<double> interp;

//   // using namespace rapidjson; 
//   // Json::FastWriter fastWriter;
//   // std::string j = fastWriter.write(opt);
//   // Document d; 
//   // const char* c = j.c_str();
//   // d.Parse(c);
//   // const rapidjson::Value& v = d["data"]; 
//   // for (rapidjson::Value::ConstMemberIterator iter = v.MemberBegin(); iter != v.MemberEnd(); ++iter) {
//   //   std::cout << (iter->name).GetString() << std::endl;
//   //   const rapidjson::Value& m = d["data"][(iter->name).GetString()]; 
    
//   //   for (rapidjson::Value::ConstMemberIterator iter2 = m.MemberBegin(); iter2 != m.MemberEnd(); ++iter2) {
//   //       std::cout << (iter2->name).GetString() << std::endl;
//   //       const rapidjson::Value& data_ = d["data"][(iter->name).GetString()][(iter2->name).GetString()];
        
//   //       std::cout << data_.IsArray() << std::endl;
//   //       // if ((iter2->value).GetType() == 4) { 
//   //       // std::string s(reinterpret_cast<char const *>(std::begin(iter2->value)),
//   //       // reinterpret_cast<char const *>(std::end(iter2->value)));
//   //   }
//   // }
// }

// //My edit: Lag

// void ConvexMPCLocomotion::initialize(){
//   for(int i = 0; i < 4; i++) firstSwing[i] = true;
//   firstRun = true;
// }

// void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
//   iterationsBetweenMPC = iterations_per_mpc;
//   dtMPC = dt * iterations_per_mpc;
// }

// void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
//   if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
//     _body_height = 0.29;
//   }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
//     _body_height = 0.45;
//   }else{
//     assert(false);
//   }

//   float x_vel_cmd, y_vel_cmd;
//   float filter(0.1);
//   if(data.controlParameters->use_rc){
//     const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
//     data.userParameters->cmpc_gait = rc_cmd->variable[0];
//     _yaw_turn_rate = -rc_cmd->omega_des[2];
//     x_vel_cmd = rc_cmd->v_des[0];
//     y_vel_cmd = rc_cmd->v_des[1] * 0.5;
//     _body_height += rc_cmd->height_variation * 0.08;
//   }else{
//     _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
//     x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
//     y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
//   }
//   _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
//   _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;

//   _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
//   _roll_des = 0.;
//   _pitch_des = 0.;

// }

// template<>
// void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  
//   // file_id.open("/home/chris/Desktop/latest/Cheetah-Software/user/MIT_Controller/original.txt", std::ios_base::app);
  
//   // t = t + 0.01;
//   // tt = std::to_string(t);
//   // std::cout << "tt " << tt << std::endl;
//   _curr_time += data.controlParameters->controller_dt + 1e-6;
//   using namespace std::chrono;
//   high_resolution_clock::time_point time2 = high_resolution_clock::now();
//   curr_duration = time2 - time1; 

//   if (firstrun) {
//     firstrun = false;
//     prev_duration = curr_duration;
//   } else {
//     duration += curr_duration - prev_duration;
//     prev_duration = curr_duration;
//   }


//   bool omniMode = false;

//   // Command Setup
//   _SetupCommand(data);
//   gaitNumber = data.userParameters->cmpc_gait;
//   if(gaitNumber >= 10) {
//     gaitNumber -= 10;
//     omniMode = true;
//   }

//   auto& seResult = data._stateEstimator->getResult();

//   // Check if transition to standing
//   if(((gaitNumber == 4) && current_gait != 4) || firstRun)
//   {
//     stand_traj[0] = seResult.position[0];
//     stand_traj[1] = seResult.position[1];
//     stand_traj[2] = 0.21;
//     stand_traj[3] = 0;
//     stand_traj[4] = 0;
//     stand_traj[5] = seResult.rpy[2];
//     world_position_desired[0] = stand_traj[0];
//     world_position_desired[1] = stand_traj[1];
//   }

//   // pick gait
//   Gait* gait = &trotting;
//   if(gaitNumber == 1)
//     gait = &bounding;
//   else if(gaitNumber == 2)
//     gait = &pronking;
//   else if(gaitNumber == 3)
//     gait = &random;
//   else if(gaitNumber == 4)
//     gait = &standing;
//   else if(gaitNumber == 5)
//     gait = &trotRunning;
//   else if(gaitNumber == 6)
//     gait = &random2;
//   else if(gaitNumber == 7)
//     gait = &random2;
//   else if(gaitNumber == 8)
//     gait = &pacing;
//   current_gait = gaitNumber;

//   gait->setIterations(iterationsBetweenMPC, iterationCounter);
//   jumping.setIterations(iterationsBetweenMPC, iterationCounter);


//   jumping.setIterations(27/2, iterationCounter);

//   //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
//   // check jump trigger
//   jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
//       data._desiredStateCommand->trigger_pressed);


//   // bool too_high = seResult.position[2] > 0.29;
//   // check jump action
//   if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
//     gait = &jumping;
//     recompute_timing(27/2);
//     _body_height = _body_height_jumping;
//     currently_jumping = true;

//   } else {
//     recompute_timing(default_iterations_between_mpc);
//     currently_jumping = false;
//   }

//   if(_body_height < 0.02) {
//     _body_height = 0.29;
//   }

//   // integrate position setpoint
//   Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
//   Vec3<float> v_des_world = 
//     omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
//   Vec3<float> v_robot = seResult.vWorld;

//   //pretty_print(v_des_world, std::cout, "v des world");

//   //Integral-esque pitche and roll compensation
//   if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
//   {
//     rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
//   }
//   if(fabs(v_robot[1]) > 0.1)
//   {
//     rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
//   }

//   rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
//   rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
//   rpy_comp[1] = v_robot[0] * rpy_int[1];
//   rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


//   for(int i = 0; i < 4; i++) {
//     pFoot[i] = seResult.position + 
//       seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
//           data._legController->datas[i].p);
//   }

//   if(gait != &standing) {
//     world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    
//   }

//   // some first time initialization
//   Vec3<float> init_body_lin;
//   Vec3<float> init_body_ang;
//   Vec3<float> init_body_vel;

//   if(firstRun)
//   {
//     world_position_desired[0] = seResult.position[0];
//     world_position_desired[1] = seResult.position[1];
//     world_position_desired[2] = seResult.rpy[2];

//     init_body_lin[0] = seResult.position[0];
//     init_body_lin[1] = seResult.position[1];
//     init_body_lin[2] = seResult.position[2];

//     init_body_ang[0] = seResult.rpy[0];
//     init_body_ang[1] = seResult.rpy[1];
//     init_body_ang[2] = seResult.rpy[2];

//     init_body_vel[0] = seResult.vBody[0];
//     init_body_vel[1] = seResult.vBody[1];
//     init_body_vel[2] = seResult.vBody[2]; 

//     for (int i(0); i<3; i++) {
//     pFoot_init_offset0[i] =  opt["data"][std::to_string(_curr_time)]["Foot position"][std::to_string(0)][i].asFloat() - pFoot[0][i];
//     pFoot_init_offset1[i] =  opt["data"][std::to_string(_curr_time)]["Foot position"][std::to_string(1)][i].asFloat() - pFoot[1][i];
//     pFoot_init_offset2[i] =  opt["data"][std::to_string(_curr_time)]["Foot position"][std::to_string(2)][i].asFloat() - pFoot[2][i];
//     pFoot_init_offset3[i] =  opt["data"][std::to_string(_curr_time)]["Foot position"][std::to_string(3)][i].asFloat() - pFoot[3][i];
    
//     }

//     // std::cout << "initial pos: " << init_body_lin[0] << init_body_lin[1] << init_body_lin[2] << std::endl;
//     // std::cout << "initial ang: " << init_body_ang[0] << init_body_ang[1] << init_body_ang[2] << std::endl;


//     for(int j = 0; j < 3; j++) {
//       using namespace std;
//       lin_pos_offset[j] = opt["data"][to_string(_curr_time)]["Base linear position"][j].asFloat() - init_body_lin[j];
//       ang_pos_offset[j] = opt["data"][to_string(_curr_time)]["Base euler position"][j].asFloat() - init_body_ang[j];
//     }

//     for(int i = 0; i < 4; i++)
//     {
//       footSwingTrajectories[i].setHeight(0.05);
//       footSwingTrajectories[i].setInitialPosition(pFoot[i]);
//       footSwingTrajectories[i].setFinalPosition(pFoot[i]);

//     }
//     firstRun = false;
//   }

//   // foot placement
//   for(int l = 0; l < 4; l++)
//     swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

//   float side_sign[4] = {-1, 1, -1, 1};
//   float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
//   //float interleave_gain = -0.13;
//   float interleave_gain = -0.2;
//   //float v_abs = std::fabs(seResult.vBody[0]);
//   float v_abs = std::fabs(v_des_robot[0]);
//   for(int i = 0; i < 4; i++)
//   {
//     if(firstSwing[i]) {
//       swingTimeRemaining[i] = swingTimes[i];
//     } else {
//       swingTimeRemaining[i] -= dt;
//     }
//     //if(firstSwing[i]) {
//     //footSwingTrajectories[i].setHeight(.05);
//     footSwingTrajectories[i].setHeight(.06);
//     Vec3<float> offset(0, side_sign[i] * .065, 0);

//     Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

//     pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
//     float stance_time = gait->getCurrentStanceTime(dtMPC, i);
//     Vec3<float> pYawCorrected = 
//       coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


//     Vec3<float> des_vel;
//     des_vel[0] = _x_vel_des;
//     des_vel[1] = _y_vel_des;
//     des_vel[2] = 0.0;

//     Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
//           + des_vel * swingTimeRemaining[i]);

//     //+ seResult.vWorld * swingTimeRemaining[i];

//     //float p_rel_max = 0.35f;
//     float p_rel_max = 0.3f;

//     // Using the estimated velocity is correct
//     //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
//     float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
//       .03f*(seResult.vWorld[0]-v_des_world[0]) +
//       (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

//     float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
//       .03f*(seResult.vWorld[1]-v_des_world[1]) +
//       (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
//     pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
//     pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
//     Pf[0] +=  pfx_rel;
//     Pf[1] +=  pfy_rel;
//     Pf[2] = -0.003;
//     //Pf[2] = 0.0;
//     footSwingTrajectories[i].setFinalPosition(Pf);
//   }

//   // calc gait
//   iterationCounter++;

//   // load LCM leg swing gains
//   Kp << 700, 0, 0,
//      0, 700, 0,
//      0, 0, 150;
//   Kp_stance = 0*Kp;


//   Kd << 7, 0, 0,
//      0, 7, 0,
//      0, 0, 7;
//   Kd_stance = Kd;
//   // gait
//   Vec4<float> contactStates = gait->getContactState();
//   Vec4<float> swingStates = gait->getSwingState();
//   int* mpcTable = gait->getMpcTable();
//   updateMPCIfNeeded(mpcTable, data, omniMode);

//   //  StateEstimator* se = hw_i->state_estimator;
//   Vec4<float> se_contactState(0,0,0,0);

// #ifdef DRAW_DEBUG_PATH
//   auto* trajectoryDebug = data.visualizationData->addPath();
//   if(trajectoryDebug) {
//     trajectoryDebug->num_points = 10;
//     trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
//     for(int i = 0; i < 10; i++) {
//       trajectoryDebug->position[i][0] = trajAll[12*i + 3];
//       trajectoryDebug->position[i][1] = trajAll[12*i + 4];
//       trajectoryDebug->position[i][2] = trajAll[12*i + 5];
//       auto* ball = data.visualizationData->addSphere();
//       ball->radius = 0.01;
//       ball->position = trajectoryDebug->position[i];
//       ball->color = {1.0, 0.2, 0.2, 0.5};
//     }
//   }
// #endif

//   for(int foot = 0; foot < 4; foot++)
//   {
//     float contactState = contactStates[foot];
//     float swingState = swingStates[foot];
//     if(swingState > 0) // foot is in swing
//     {
//       if(firstSwing[foot])
//       {
//         firstSwing[foot] = false;
//         footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
//       }

// #ifdef DRAW_DEBUG_SWINGS
//       auto* debugPath = data.visualizationData->addPath();
//       if(debugPath) {
//         debugPath->num_points = 100;
//         debugPath->color = {0.2,1,0.2,0.5};
//         float step = (1.f - swingState) / 100.f;
//         for(int i = 0; i < 100; i++) {
//           footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
//           debugPath->position[i] = footSwingTrajectories[foot].getPosition();
//         }
//       }
//       auto* finalSphere = data.visualizationData->addSphere();
//       if(finalSphere) {
//         finalSphere->position = footSwingTrajectories[foot].getPosition();
//         finalSphere->radius = 0.02;
//         finalSphere->color = {0.6, 0.6, 0.2, 0.7};
//       }
//       footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
//       auto* actualSphere = data.visualizationData->addSphere();
//       auto* goalSphere = data.visualizationData->addSphere();
//       goalSphere->position = footSwingTrajectories[foot].getPosition();
//       actualSphere->position = pFoot[foot];
//       goalSphere->radius = 0.02;
//       actualSphere->radius = 0.02;
//       goalSphere->color = {0.2, 1, 0.2, 0.7};
//       actualSphere->color = {0.8, 0.2, 0.2, 0.7};
// #endif
       
//       // My edit 
//       // auto duration = opt[tt]["Duration"][foot];
//       // int ptr = 0; 
//       // std::map<int,float> store;
//       // for (int j(0); j < (signed)opt[tt]["Duration"][foot].size()-1; j++) {
//       //   auto curr = duration[j].asFloat();
//       //   store.insert(std::pair<int, float>(j,curr));
//       //   curr += curr + duration[j+1].asFloat();
//       // }
//       // std::map<int,float>::iterator itr;
//       // // Print the map
//       // for (itr = store.begin(); itr !=store.end(); ++itr) {
//       //   std::cout << '\t' << itr->first << '\t' << itr->second << '\n';
//       // }

//       // for (itr = store.begin(); itr !=store.end(); ++itr) {
//       //   if (t <= itr->second) {
//       //     ptr = itr->first; 
//       //   }
//       // }

//       footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
//       // float constact_state = opt["Contact State"][std::to_string(foot)].asFloat();
//       // Vec4<float> my_swingTimes;
//       // for (int i(0); i<4; i++) {
//       //   my_swingTimes[i] = duration[ptr].asFloat();
//       // }
//       // footSwingTrajectories[foot].computeSwingTrajectoryBezier(constact_state, my_swingTimes[foot]); 
//       // std::cout << "traj" << std::endl  << footSwingTrajectories << std::endl;


//       //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
//       //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
//       //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);
   
//       // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition(); 
//       // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
//       // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
//       //   - data._quadruped->getHipLocation(foot);
//       // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

//       // Update for WBC
//       // My edit 
//       // pFoot_des[foot] = pDesFootWorld;
//       // vFoot_des[foot] = vDesFootWorld;
//       // aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
  
    
//       for (int i(0); i < 3; i++) {
//         // Get the difference between Towr init position and cheetah init pos
//         using namespace std;
//         pFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(0)][i].asFloat() + pFoot_init_offset0[i];
//         vFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(0)][i].asFloat();
//         aFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(0)][i].asFloat();


//         pFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(1)][i].asFloat() + pFoot_init_offset1[i];
//         vFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(1)][i].asFloat();
//         aFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(1)][i].asFloat();

//         pFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(2)][i].asFloat() + pFoot_init_offset2[i];
//         vFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(2)][i].asFloat();
//         aFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(2)][i].asFloat();

//         pFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(3)][i].asFloat() + pFoot_init_offset3[i];
//         vFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(3)][i].asFloat();
//         aFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(3)][i].asFloat();
//       }

//       Vec3<float> pDesLeg = seResult.rBody * (pFoot_des[foot] - seResult.position) - data._quadruped->getHipLocation(foot);
//       Vec3<float> vDesLeg = seResult.rBody * (vFoot_des[foot] - seResult.vWorld);




//       for (int k(0); k<4; k++){
//         std::cout << "TIME: " << tt << std::endl;
//         std::cout << "Foot Pos" << "leg" << k << pFoot_des[k] << std::endl;
//         std::cout << "Foot Vel" << "leg" << k << vFoot_des[k] << std::endl;
//         std::cout << "Foot Acc" << "leg" << k << aFoot_des[k] << std::endl;
//       }

 
//       if(!data.userParameters->use_wbc){
//         // Update leg control command regardless of the usage of WBIC
//         data._legController->commands[foot].pDes = pDesLeg;
//         data._legController->commands[foot].vDes = vDesLeg;
//         data._legController->commands[foot].kpCartesian = Kp;
//         data._legController->commands[foot].kdCartesian = Kd;
//       }
//     }
//     else // foot is in stance
//     {
//       firstSwing[foot] = true;

// #ifdef DRAW_DEBUG_SWINGS
//       auto* actualSphere = data.visualizationData->addSphere();
//       actualSphere->position = pFoot[foot];
//       actualSphere->radius = 0.02;
//       actualSphere->color = {0.2, 0.2, 0.8, 0.7};
// #endif

//       // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
//       // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
//       // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
//       // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
//       //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
//       //My edit 
//         for (int i(0); i < 3; i++) {
//         using namespace std;
//         pFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(0)][i].asFloat() + pFoot_init_offset0[i];
//         vFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(0)][i].asFloat();
//         aFoot_des[0][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(0)][i].asFloat();


//         pFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(1)][i].asFloat() + pFoot_init_offset1[i];
//         vFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(1)][i].asFloat();
//         aFoot_des[1][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(1)][i].asFloat();

//         pFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(2)][i].asFloat() + pFoot_init_offset2[i];
//         vFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(2)][i].asFloat();
//         aFoot_des[2][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(2)][i].asFloat();

//         pFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot position"][std::to_string(3)][i].asFloat() + pFoot_init_offset3[i];
//         vFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot velocity"][std::to_string(3)][i].asFloat();
//         aFoot_des[3][i] = opt["data"][to_string(_curr_time)]["Foot acceleration"][std::to_string(3)][i].asFloat();
//       }

//       Vec3<float> pDesLeg = seResult.rBody * (pFoot_des[foot] - seResult.position) - data._quadruped->getHipLocation(foot);
//       Vec3<float> vDesLeg = seResult.rBody * (vFoot_des[foot] - seResult.vWorld);




//       if(!data.userParameters->use_wbc){
//         data._legController->commands[foot].pDes = pDesLeg;
//         data._legController->commands[foot].vDes = vDesLeg;
//         data._legController->commands[foot].kpCartesian = Kp_stance;
//         data._legController->commands[foot].kdCartesian = Kd_stance;

//         data._legController->commands[foot].forceFeedForward = f_ff[foot];
//         data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

//         //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
//         //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
//         // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
//       }else{ // Stance foot damping
//         data._legController->commands[foot].pDes = pDesLeg;
//         data._legController->commands[foot].vDes = vDesLeg;
//         data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
//         data._legController->commands[foot].kdCartesian = Kd_stance;
//       }
//       //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
//       se_contactState[foot] = contactState;
//       // std::cout << "contact state" << foot << ": " << contactState << std::endl;


//       // Update for WBC
//       //Fr_des[foot] = -f_ff[foot];
//     }
//   }
 
//   // se->set_contact_state(se_contactState); todo removed
//   data._stateEstimator->setContactPhase(se_contactState); 
//   std::cout << "contact state to estimator " << se_contactState[0] << se_contactState[1] << se_contactState[2]  << se_contactState[3] << std::endl;

//   // Update For WBC
//   // My edit 
//   using namespace std;
//   pBody_des[0] = opt["data"][to_string(_curr_time)]["Base linear position"][0].asFloat() + lin_pos_offset[0];
//   pBody_des[1] = opt["data"][to_string(_curr_time)]["Base linear position"][1].asFloat() + lin_pos_offset[1];
//   // pBody_des[2] = opt["data"][to_string(_curr_time)]["Base linear position"][2].asFloat() + lin_pos_offset[2];
//   pBody_des[2] = _body_height;
 
//   vBody_des[0] = opt["data"][to_string(_curr_time)]["Base linear velocity"][0].asFloat();
//   vBody_des[1] = opt["data"][to_string(_curr_time)]["Base linear velocity"][1].asFloat();
//   vBody_des[2] = 0.;
//   // vBody_des[2] = opt["data"][to_string(_curr_time)]["Base linear velocity"][2].asFloat();

//   aBody_des[0] = opt["data"][to_string(_curr_time)]["Base linear acceleration"][0].asFloat();
//   aBody_des[1] = opt["data"][to_string(_curr_time)]["Base linear acceleration"][1].asFloat();
//   aBody_des[2] = opt["data"][to_string(_curr_time)]["Base linear acceleration"][2].asFloat();;

//   pBody_RPY_des[0] = opt["data"][to_string(_curr_time)]["Base euler position"][0].asFloat() + ang_pos_offset[0];
//   pBody_RPY_des[1] = opt["data"][to_string(_curr_time)]["Base euler position"][1].asFloat() + ang_pos_offset[1];
//   pBody_RPY_des[2] = opt["data"][to_string(_curr_time)]["Base euler position"][2].asFloat() + ang_pos_offset[2];


//   vBody_Ori_des[0] = opt["data"][to_string(_curr_time)]["Base euler velocity"][0].asFloat();
//   vBody_Ori_des[1] = opt["data"][to_string(_curr_time)]["Base euler velocity"][1].asFloat();
//   vBody_Ori_des[2] = opt["data"][to_string(_curr_time)]["Base euler velocity"][2].asFloat();

  
 
//   contact_state = gait->getContactState();
//   // std::cout << "contact_state" << contact_state << std::endl;



//   // pBody_des[0] = world_position_desired[0];
//   // pBody_des[1] = world_position_desired[1];
//   // pBody_des[2] = _body_height;

//   // vBody_des[0] = v_des_world[0];
//   // vBody_des[1] = v_des_world[1];
//   // vBody_des[2] = 0.;
  

//   // aBody_des.setZero();

//   // pBody_RPY_des[0] = 0.;
//   // pBody_RPY_des[1] = 0.; 
//   // pBody_RPY_des[2] = _yaw_des;

//   // vBody_Ori_des[0] = 0.;
//   // vBody_Ori_des[1] = 0.;
//   // vBody_Ori_des[2] = _yaw_turn_rate; 

//   // contact_state = gait->getContactState();


//   // std::cout << "time!time!time!" << std::endl << tt << std::endl; 
//   // std::cout << "pBody_des" << std::endl << pBody_des << std::endl;
//   // std::cout << "vBody_des" << std::endl << vBody_des << std::endl;
//   // std::cout << "aBody_des" << std::endl << aBody_des << std::endl;
//   // std::cout << "pBody_RPY_des" << std::endl << pBody_RPY_des << std::endl;
//   // std::cout << "vBody_Ori_des" << std::endl << vBody_Ori_des << std::endl;
//   // std::cout << "Contact State" << std::endl << contact_state << std::endl;
  

//   // // Print state estimation 
//   // std::cout << "State Estimate: pBody" << std::endl << seResult.position << std::endl;
//   // std::cout << "State Estimate: vBody" << std::endl << seResult.vBody << std::endl;
//   // std::cout << "State Estimate: orientation" << std::endl << seResult.orientation << std::endl;
//   // std::cout << "State Estimate: omegaBody" << std::endl << seResult.omegaBody << std::endl;
//   // // std::cout << "State Estimate: rBdoy" << std::endl << seResult.rBody << std::endl;
//   // std::cout << "State Estimate: rpy" << std::endl << seResult.rpy << std::endl;

//   // END of WBC Update


// }

// template<>
// void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
//   (void)data;
//   printf("call to old CMPC with double!\n");

// }

// void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
//   //iterationsBetweenMPC = 30;
//   if((iterationCounter % iterationsBetweenMPC) == 0)
//   {
//     auto seResult = data._stateEstimator->getResult();
//     float* p = seResult.position.data();

//     Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
//     Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
//     //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


//     //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

//     if(current_gait == 4)
//     {
//       float trajInitial[12] = {
//         _roll_des,
//         _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
//         (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
//         (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
//         (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
//         (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
//         0,0,0,0,0,0};

//       for(int i = 0; i < horizonLength; i++)
//         for(int j = 0; j < 12; j++)
//           trajAll[12*i+j] = trajInitial[j];
//     }

//     else
//     {
//       const float max_pos_error = .1;
//       float xStart = world_position_desired[0];
//       float yStart = world_position_desired[1];

//       if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
//       if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

//       if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
//       if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

//       world_position_desired[0] = xStart;
//       world_position_desired[1] = yStart;

//       float trajInitial[12] = {(float)rpy_comp[0],  // 0
//         (float)rpy_comp[1],    // 1
//         _yaw_des,    // 2
//         //yawStart,    // 2
//         xStart,                                   // 3
//         yStart,                                   // 4
//         (float)_body_height,      // 5
//         0,                                        // 6
//         0,                                        // 7
//         _yaw_turn_rate,  // 8
//         v_des_world[0],                           // 9
//         v_des_world[1],                           // 10
//         0};                                       // 11

//       for(int i = 0; i < horizonLength; i++)
//       {
//         for(int j = 0; j < 12; j++)
//           trajAll[12*i+j] = trajInitial[j];

//         if(i == 0) // start at current position  TODO consider not doing this
//         {
//           //trajAll[3] = hw_i->state_estimator->se_pBody[0];
//           //trajAll[4] = hw_i->state_estimator->se_pBody[1];
//           trajAll[2] = seResult.rpy[2];
//         }
//         else
//         {
//           trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
//           trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
//           trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
//         }
//       }
//     }
//     Timer solveTimer;

//     if(_parameters->cmpc_use_sparse > 0.5) {
//       solveSparseMPC(mpcTable, data);
//     } else {
//       solveDenseMPC(mpcTable, data);
//     }
//     //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
//   }

// }

// void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
//   auto seResult = data._stateEstimator->getResult();

//   //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

//   float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

//   //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
//   float yaw = seResult.rpy[2];
//   float* weights = Q;
//   float alpha = 4e-5; // make setting eventually
//   //float alpha = 4e-7; // make setting eventually: DH
//   float* p = seResult.position.data();
//   float* v = seResult.vWorld.data();
//   float* w = seResult.omegaWorld.data();
//   float* q = seResult.orientation.data();

//   float r[12];
//   for(int i = 0; i < 12; i++)
//     r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

//   //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

//   if(alpha > 1e-4) {
//     std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
//     alpha = 1e-5;
//   }

//   Vec3<float> pxy_act(p[0], p[1], 0);
//   Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
//   //Vec3<float> pxy_err = pxy_act - pxy_des;
//   float pz_err = p[2] - _body_height;
//   Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

//   Timer t1;
//   dtMPC = dt * iterationsBetweenMPC;
//   setup_problem(dtMPC,horizonLength,0.4,120);
//   //setup_problem(dtMPC,horizonLength,0.4,650); //DH
//   update_x_drag(x_comp_integral);
//   if(vxy[0] > 0.3 || vxy[0] < -0.3) {
//     //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
//     x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
//   }

//   //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

//   update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
//       _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
//   //t1.stopPrint("Setup MPC");

//   Timer t2;
//   //cout << "dtMPC: " << dtMPC << "\n";
//   update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
//   //t2.stopPrint("Run MPC");
//   //printf("MPC Solve time %f ms\n", t2.getMs());  

//   for(int leg = 0; leg < 4; leg++)
//   {
//     Vec3<float> f;
//     for(int axis = 0; axis < 3; axis++) {
//       // f[axis] = get_solution(leg*3 + axis);
//       f[axis] = opt["data"][std::to_string(_curr_time)]["Contact forces"][std::to_string(leg)][axis].asFloat();
//     } 
    
//     // printf("Leg force [%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

//     f_ff[leg] = -seResult.rBody * f;
//     // Update for WBC
//     Fr_des[leg] = f;
//     // std::cout<<"Contact forces" << f << std::endl;
//   }
// }

// void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
//   // X0, contact trajectory, state trajectory, feet, get result!
//   (void)mpcTable;
//   (void)data;
//   auto seResult = data._stateEstimator->getResult();

//   std::vector<ContactState> contactStates;
//   for(int i = 0; i < horizonLength; i++) {
//     contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
//   }

//   for(int i = 0; i < horizonLength; i++) {
//     for(u32 j = 0; j < 12; j++) {
//       _sparseTrajectory[i][j] = trajAll[i*12 + j];
//     }
//   }

//   Vec12<float> feet;
//   for(u32 foot = 0; foot < 4; foot++) {
//     for(u32 axis = 0; axis < 3; axis++) {
//       feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
//     }
//   }

//   _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
//   _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
//   _sparseCMPC.setStateTrajectory(_sparseTrajectory);
//   _sparseCMPC.setFeet(feet);
//   _sparseCMPC.run();

//   Vec12<float> resultForce = _sparseCMPC.getResult();

//   for(u32 foot = 0; foot < 4; foot++) {
//     Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
//     //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
//     f_ff[foot] = -seResult.rBody * force;
//     Fr_des[foot] = force;
//   }
// }

// void ConvexMPCLocomotion::initSparseMPC() {
//   Mat3<double> baseInertia;
//   baseInertia << 0.07, 0, 0,
//               0, 0.26, 0,
//               0, 0, 0.242;
//   double mass = 9;
//   double maxForce = 120;

//   std::vector<double> dtTraj;
//   for(int i = 0; i < horizonLength; i++) {
//     dtTraj.push_back(dtMPC);
//   }

//   Vec12<double> weights;
//   weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
//   //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

//   _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
//   _sparseCMPC.setFriction(0.4);
//   _sparseCMPC.setWeights(weights, 4e-5);
//   _sparseCMPC.setDtTrajectory(dtTraj);

//   _sparseTrajectory.resize(horizonLength);
// }


#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
    _body_height = 0.29;
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    _body_height = 0.45;
  }else{
    assert(false);
  }

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  if(data.controlParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
  }else{
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  }
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;

  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if(_body_height < 0.02) {
    _body_height = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    footSwingTrajectories[i].setHeight(.06);
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = -0.003;
    //Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update


}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,120);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

