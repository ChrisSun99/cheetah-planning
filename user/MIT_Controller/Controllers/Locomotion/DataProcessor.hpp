// #include "FSM_States/FSM_State_Loco.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <json/writer.h>
#include <json/value.h>
#include <json/reader.h>
#include <cppTypes.h>
#include <boost/format.hpp> 
#include "Controllers/LegController.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"
#include "Utilities/utilities.h"

class DataProcessor {
 public:
  DataProcessor();
  Eigen::VectorXd ComputeJointVelocities(float t);
  // Vec12<double> ComputeJointTorques(float t);
  void WriteAsDatFile(Eigen::MatrixXd result, int i);
  // Mat3<T> computeLegJacobian(Quadruped<T>& quad, Vec3<T>& q, Mat3<T> J, int leg);
  // Vec3<T> computeLegPosition(Quadruped<T>& quad, Vec3<T>& q, Vec3<T> p, int leg);
  

private:
    FloatingBaseModel<double> cheetahModel;
    FBModelState<double> x;
    Vec12<double> jointTorques;
    SVec<double> fReturn;
    Json::Value opt;
    Json::Reader reader; 
    float t; // current timestamp
    std::ofstream file_in; 
    std::ofstream file_out; 
    Vec12<double> footForces; 
    

    
 
};



