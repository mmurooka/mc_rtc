#include "MyFirstFsmController.h"

MyFirstFsmController::MyFirstFsmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  LOG_SUCCESS("MyFirstFsmController init done " << this)
}

bool MyFirstFsmController::run()
{
  return mc_control::fsm::Controller::run();
}

void MyFirstFsmController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


