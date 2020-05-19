#include "MyFirstFsmController_Initial.h"

#include "../MyFirstFsmController.h"

void MyFirstFsmController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void MyFirstFsmController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MyFirstFsmController &>(ctl_);
}

bool MyFirstFsmController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MyFirstFsmController &>(ctl_);
  output("OK");
  return true;
}

void MyFirstFsmController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MyFirstFsmController &>(ctl_);
}

EXPORT_SINGLE_STATE("MyFirstFsmController_Initial", MyFirstFsmController_Initial)
