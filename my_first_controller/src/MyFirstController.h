#pragma once

#include <mc_control/mc_controller.h>
// Include the CoM task header (header)
#include <mc_tasks/CoMTask.h>
// Include the EF task header (header)
#include <mc_tasks/EndEffectorTask.h>
// Get the task loader in there
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rbdyn/RobotLoader.h>
// In the header
#include <mc_tasks/SurfaceTransformTask.h>

// Brings in every type we need
#include <mc_rtc/gui/plot.h>

#include "api.h"


// In the header
enum DoorPhase
  {
    APPROACH = 0,
    HANDLE,
    OPEN
  };

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    void switch_target();

    void switch_com_target();

    void switch_phase();

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

    // In the header
    std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics;
    std::shared_ptr<mc_tasks::PostureTask> doorPosture;

    // A private property of our controller
    DoorPhase phase = APPROACH;

    // In the private members
    std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask;
};
