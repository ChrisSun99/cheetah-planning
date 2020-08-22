
#include "BackFlip/DataReader.hpp"
#include "BackFlip/DataReadCtrl.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>
#include <Locomotion/DataProcessor.hpp>

template <typename T>
class Locomotion : public DataReadCtrl<T> {
 public:
  Locomotion(DataReader*, float _dt);
  virtual ~Locomotion();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);


  virtual void _update_joint_command();
  void computeCommand(LegControllerCommand<T>* cmd, LegControllerData<T>* data); 

//  private:
  // DataProcessor data_processor;
};

