/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

struct MC_CONTROL_DLLAPI LIPMStabilizerController : public mc_control::fsm::Controller
{
  LIPMStabilizerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void supported_robots(std::vector<std::string> & out) const override
  {
    out = {"jvrc1", "hrp2_drc", "hrp4", "hrp5_p"};
  }

private:
  mc_rtc::Configuration config_;
};
