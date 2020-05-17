#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    void switch_target();

    mc_rtc::Configuration config_;

    // Added to MyFirstController.h in the private members
    int jointIndex = 0;
    bool goingLeft = true;
};