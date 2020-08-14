#include <Utilities/Timer.h>
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
#include "Dynamics/spatial.h"
#include "Utilities/utilities.h"
#include "DataProcessor.hpp"

// This class is to convert the json file generated by TOWR to a .dat file 
// and compute q, qd and tau required by the PD controller. 

template <typename Iterable>
Json::Value iterable2json(Iterable const& cont);
Eigen::VectorXd convertJson2Arr(Eigen::Vector4d arr, Json::Value &j);
Vec3<double> convertJson2Vec(Vec3<double> vec, Json::Value &j);
Eigen::Matrix3d convertJson2Mat(Eigen::Matrix3d mat, Json::Value &js);
Eigen::VectorXd convertJson2Xd(Eigen::VectorXd jointPos, Json::Value &js);
SVec<double> convertJson2SVec(SVec<double> arr, Json::Value &j1, Json::Value &j2);

DataProcessor::DataProcessor()  {
  std::cout << "canoot print" << std::endl;
  std::ifstream file_in("/home/chris/Desktop/latest/Cheetah-Software/user/MIT_Controller/ros_output.json", std::ifstream::binary);
  if (!file_in.is_open()){
       std::cout << "Error opening file"<< std::endl;
       exit (1);
  }
  Json::Reader reader; 
  bool parseSuccessful = reader.parse(file_in, opt);
  if (!parseSuccessful) {
    std::cout
     << "ERROR DURING PARSING JSON FILE.";
  }
  std::cout << "HHHHHHH" << std::endl;
  Eigen::MatrixXd result; 
  result.resize(1001, 60);
  int i = 0; 
  using namespace std; 
  t = 0. + 1e-6;
  while (t < 2) {

    Vec3<double> pos, omegaBody, vBody; 
    Eigen::Matrix3d mat; 
    Eigen::VectorXd jointAng;
    jointAng.resize(12);
    DVec<double> q(12);
    DVec<double> dq(12);
    DVec<double> tauref(12);

  cheetahModel = buildMiniCheetah<double>().buildModel();

  // set state
  x.bodyOrientation = rotationMatrixToQuaternion(convertJson2Mat(mat, opt["data"][to_string(t)]["Quat"])); // Quat 
  x.bodyPosition    = convertJson2Vec(pos, opt["data"][to_string(t)]["Base linear position"]);
  x.bodyVelocity.head(3) = convertJson2Vec(omegaBody, opt["data"][to_string(t)]["Base euler velocity"]); //omegaBody
  x.bodyVelocity.tail(3) = convertJson2Vec(vBody, opt["data"][to_string(t)]["Base linear velocity"]);  //vBody
  x.q = convertJson2Xd(jointAng, opt["data"][to_string(t)]["Joint position"]);
  x.qd = ComputeJointVelocities(t);
  cheetahModel.setState(x);

  // Construct state derivative
  DVec<double> qdd(12);
  // qdd << -0.7924, -0.2205, -0.9163, -1.6136, -0.4328, -1.6911, -2.9878, -0.9358,
  //     -2.6194, -3.3773, -1.3235, -3.1598;
  // qdd *= 1000;
  qdd.setZero();
  
  SVec<double> vbd = convertJson2SVec(vbd, opt["data"][to_string(t)]["Base linear acceleration"], opt["data"][to_string(t)]["Base angular acceleration"]);
  FBModelStateDerivative<double> dx;
  dx.dBodyVelocity = vbd;
  dx.qdd = qdd;
  cheetahModel.setDState(dx);
  qdd(0) = (cheetahModel.getLinearAcceleration(0))(0);
  qdd(1) = (cheetahModel.getLinearAcceleration(0))(1);
  qdd(2) = (cheetahModel.getLinearAcceleration(0))(2);
  qdd(3) = (cheetahModel.getLinearAcceleration(1))(0);
  qdd(4) = (cheetahModel.getLinearAcceleration(1))(1);
  qdd(5) = (cheetahModel.getLinearAcceleration(1))(2);
  qdd(6) = (cheetahModel.getLinearAcceleration(2))(0);
  qdd(7) = (cheetahModel.getLinearAcceleration(2))(1);
  qdd(8) = (cheetahModel.getLinearAcceleration(2))(2);
  qdd(9) = (cheetahModel.getLinearAcceleration(3))(0);
  qdd(10) = (cheetahModel.getLinearAcceleration(3))(1);
  qdd(11) = (cheetahModel.getLinearAcceleration(3))(2);

  // Compute ID two different ways
  DVec<double> genForce = cheetahModel.inverseDynamics(dx);
  DVec<double> jointTorques = genForce.tail(12);

  // LOAD FOOT FORCE 
  Eigen::Vector3d foot0, foot1, foot2, foot3; 

  Eigen::Vector3d f1, f2, f3, f4; 
  f1 = convertJson2Vec(foot0, opt["data"][to_string(t)]["Contact forces"]["0"]); 
  f2 = convertJson2Vec(foot1, opt["data"][to_string(t)]["Contact forces"]["1"]); 
  f3 = convertJson2Vec(foot2, opt["data"][to_string(t)]["Contact forces"]["2"]); 
  f4 = convertJson2Vec(foot3, opt["data"][to_string(t)]["Contact forces"]["3"]); 
  for (int i = 0; i < 3; i++) {
    footForces[i] = f1[i];
    footForces[i + 3] = f2[i];
    footForces[i + 6] = f3[i];
    footForces[i + 9] = f4[i];
  }
  
  WriteAsDatFile(result, i);
  i += 1; 
  t = t + 0.002;
  }
  
  // result.resize(1, result.size());
  // Eigen::VectorXd d(Eigen::Map<Eigen::VectorXd>(result.data(),result.cols()*result.rows()));
  // FILE* f = fopen("/home/chris/Desktop/latest/Cheetah-Software/config/locomotion.dat","w");
  // fwrite(d.data(), sizeof(float), result.size(), f);
  // fclose(f);

  
}


Eigen::VectorXd
convertJson2Arr(Eigen::Vector4d arr, Json::Value &j) 
{
  for (int i = 0; i < arr.size(); ++i) {
    arr[i] = j[i].asDouble();
  }
  return arr;
}

SVec<double>
convertJson2SVec(SVec<double> arr, Json::Value &j1, Json::Value &j2) 
{
  for (int i = 0; i < 3; ++i) {
    arr[i] = j1[i].asFloat();
  }
   for (int i = 3; i < 6; ++i) {
    arr[i] = j2[i].asFloat();
  }
  return arr;
}

Vec3<double>
convertJson2Vec(Vec3<double> arr, Json::Value &j) 
{
  for (int i = 0; i < arr.size(); ++i) {
    arr[i] = j[i].asFloat();
  }
  return arr; 
}

Eigen::Matrix3d
convertJson2Mat(Eigen::Matrix3d mat, Json::Value &js) 
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat(i,j) = js[i * 3 + j].asFloat();
    }
  }
  return mat; 
}

Eigen::VectorXd 
convertJson2Xd(Eigen::VectorXd jointPos, Json::Value &js)
{
  for (int i = 0; i < jointPos.size(); i++) {
    jointPos[i] = js[i].asFloat();
  }
  return jointPos;
}

Eigen::VectorXd
DataProcessor::ComputeJointVelocities(float t) {
  
    Eigen::VectorXd v;
    v.resize(12);
    Eigen::Vector3d vec0 = cheetahModel.getLinearVelocity(0);
    Eigen::Vector3d vec1 = cheetahModel.getLinearVelocity(1);
    Eigen::Vector3d vec2 = cheetahModel.getLinearVelocity(2);
    Eigen::Vector3d vec3 = cheetahModel.getLinearVelocity(3);
    for (int i = 0; i < 3; i++) {
      v(i) = vec0(i);
      v(i + 3)  = vec1(i);
      v(i + 6)  = vec2(i);
      v(i + 9)  = vec3(i);
    }
    
    return v; 
}

// Vec12<double>
// DataProcessor::ComputeJointTorques(float t) {
//     // Construct state derivative
//     SVec<double> vbd = convertJson2SVec(vbd, opt["data"][to_string(t)]["Base linear acceleration"], opt["data"][to_string(t)]["Base angular acceleration"]);
//     // DVec<double> qdd(12);
//     // Fill in joint acceleration values 
//     // _model.forwardAccelerationKinematics();
//     // Mat3<double> R = _model.getOrientation(0);
//     // Vec3<double> a = R * spatialToLinearAcceleration(_model._a[0], _model._v[0], Vec3<double>::Zero());
//     DVec<double> qdd(12);
//     qdd << -0.7924, -0.2205, -0.9163, -1.6136, -0.4328, -1.6911, -2.9878, -0.9358,
//         -2.6194, -3.3773, -1.3235, -3.1598;
//     qdd *= 1000;

//     FBModelStateDerivative<double> dx;
//     dx.dBodyVelocity = vbd;
//     dx.qdd = qdd;
//     // Run RNEA inverse dynamics
//     Vec18<double> generalizedForce = cheetahModel.inverseDynamics(dx); // [base_force ; joint_torques]
//     fReturn = generalizedForce.head(6);
//     jointTorques = generalizedForce.tail(12); // prune away base force
//     return jointTorques;
// }

void 
DataProcessor::WriteAsDatFile(Eigen::MatrixXd result, int i) {
  // file_out.open("/home/chris/Desktop/latest/Cheetah-Software/config/locomotion.dat", ios::app);
  // if (!file_out) {
  //   cerr << "ERROR: FILE COULD NOT BE OPENED." << endl;
  //   exit(1);
  // }
      std::cout << "i: " << i << std::endl;

  
    result(i, 0) = x.bodyPosition[0];
    result(i, 1) = x.bodyPosition[1];
    result(i, 2) =  x.bodyPosition[2];
    result(i, 3) = x.bodyOrientation[0];
    result(i, 4) = x.bodyOrientation[1];
    result(i, 5) = x.bodyOrientation[2];
    result(i, 6) =  (x.q)[0];
    result(i, 7) = (x.q)[1];
    result(i, 8) = (x.q)[2];
    result(i, 9) = (x.q)[3];
    result(i, 10) = (x.q)[4];
    result(i, 11) = (x.q)[5];
    result(i, 12) =(x.q)[6];
    result(i, 13) = (x.q)[7];
    result(i, 14) =  (x.q)[8];
    result(i, 15) = (x.q)[9];
    result(i, 16) = (x.q)[10];
    result(i, 17) = (x.q)[11];
    result(i, 18)= x.bodyVelocity[0];
    result(i, 19) = x.bodyVelocity[1];
    result(i, 20) = x.bodyVelocity[2];
    result(i, 21) = x.bodyOrientation[0];
    result(i, 22) = x.bodyOrientation[1];
    result(i, 23) = x.bodyOrientation[2];
    result(i, 24) =(x.qd)[0];
    result(i, 25) = (x.qd)[1]; 
    result(i, 26) = (x.qd)[2];
    result(i, 27) = (x.qd)[3];
    result(i, 28) = (x.qd)[4];
    result(i, 29) =(x.qd)[5];
    result(i, 30) = (x.qd)[6];
    result(i, 31) = (x.qd)[7];
    result(i, 32) = (x.qd)[8];
    result(i, 33) = (x.qd)[9];
    result(i, 34) = (x.qd)[10] ;
    result(i, 35) = (x.qd)[11];
    result(i, 36) =jointTorques[0] ;
    result(i, 37) = jointTorques[1] ;
    result(i, 38) = jointTorques[2];
    result(i, 39) = jointTorques[3] ;
    result(i, 40) = jointTorques[4] ;
    result(i, 41) =jointTorques[5];
    result(i, 42) = jointTorques[6]; 
    result(i, 43) = jointTorques[7];
    result(i, 44) =jointTorques[8];
    result(i, 45) = jointTorques[9];
    result(i, 46) =jointTorques[10];
    result(i, 47) =jointTorques[11];
    result(i, 48) =footForces[0]; 
    result(i, 49) =footForces[1];
    result(i, 50) =footForces[2];
    result(i, 51) = footForces[3] ;
    result(i, 52) =footForces[4] ;
    result(i, 53) =footForces[5] ;
    result(i, 54) =footForces[6];
    result(i, 55) = footForces[7]; 
    result(i, 56) =footForces[8];
    result(i, 57) =footForces[9];
    result(i, 58) =footForces[10] ;
    result(i, 59) =footForces[11] ;
    std::cout << "done" << std::endl;
  

  // file_out << (float)x.bodyPosition[0] << " " << (float)x.bodyPosition[1] << " " << (float)x.bodyPosition[2] <<  " " << (float)x.bodyOrientation[0] << " " << x.bodyOrientation[1] << " " << x.bodyOrientation[2] <<  " ";
  // file_out << (x.q)[0] << " " << (x.q)[1] << " " << (x.q)[2] << " " << (x.q)[3] << " " << (x.q)[4] << " " << (x.q)[5] << " " << (x.q)[6] <<  " " << (x.q)[7] <<  " " << (x.q)[8] <<  " " << (x.q)[9] << " " << (x.q)[10] << " " << (x.q)[11] << " ";
  // file_out << x.bodyVelocity[0] <<  " " << x.bodyVelocity[1] << " " << x.bodyVelocity[2] <<  " " << x.bodyOrientation[0] << " " << x.bodyOrientation[1] << " " << x.bodyOrientation[2] << " ";
  // file_out << (x.qd)[0] << " " << (x.qd)[1] << " " << (x.qd)[2] << " " << (x.qd)[3] << " " << (x.qd)[4] << " " << (x.qd)[5] << " " << (x.qd)[6] << " " << (x.qd)[7] << " " << (x.qd)[8] << " " << (x.qd)[9] << " " << (x.qd)[10] << " " << (x.qd)[11] << " ";
  // file_out << jointTorques[0] << " " << jointTorques[1] << " " << jointTorques[2] << " " << jointTorques[3] << " " << jointTorques[4] << " " << jointTorques[5] << " " << jointTorques[6] << " " << jointTorques[7] << " ";
  // file_out << jointTorques[8] << " " << jointTorques[9] << " " << jointTorques[10] << " " << jointTorques[11] << " ";
  // file_out << footForces[0] << " " << footForces[1] << " " << footForces[2] << " " << footForces[3] << " " << footForces[4] << " " << footForces[5] << " " << footForces[6] << " " << footForces[7] << " " << footForces[8] << " ";
  // file_out << footForces[9] << " " << footForces[10] << " " << footForces[11];
  // file_out.close();
}

template <typename Iterable> 
Json::Value iterable2json(Iterable const& cont) {
  Json::Value v;
  // for (auto&& element: cont) {
  //   v.append(element);
  // }
  for (auto iteration = 0; iteration < cont.size(); iteration++) {
    v.append(cont[iteration]);
  }
  return v; 
}


/**
template <typename T>
void computeLegJacobian(Quadruped<T>& quad, Vec3<T>& q, Mat3<T> J, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J[0, 0] = 0;
    J[0, 1] = l3 * c23 + l2 * c2;
    J[0, 2] = l3 * c23;
    J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2;
    J[1, 2] = -l3 * s1 * s23;
    J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2;
    J[2, 2] = l3 * c1 * s23;
  }

 return J; 
}


Vec3<T> ComputeLegPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T> p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;
    if (p) {
    p[0] = l3 * s23 + l2 * s2;
    p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
  return p; 
}

*/

