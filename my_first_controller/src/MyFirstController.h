#pragma once

#include <mc_control/mc_controller.h>
// Include the CoM task header (header)
#include <mc_tasks/CoMTask.h>
// Include the EF task header (header)
#include <mc_tasks/EndEffectorTask.h>
// Get the task loader in there
#include <mc_tasks/MetaTaskLoader.h>

#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    void switch_target();

    void switch_com_target();

    mc_rtc::Configuration config_;

    // Added to MyFirstController.h in the private members
    int jointIndex = 0;
    bool goingLeft = true;

    // In the class private members (header)
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    Eigen::Vector3d comZero;
    bool comDown = true;

    // In the class private members (header)
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
};
