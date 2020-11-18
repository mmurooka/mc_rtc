/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_body6d_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCBody6dController::MCBody6dController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(*compoundJointConstraint);
  solver().addTask(postureTask.get());
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"), mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});
  }
  else if(robot().mb().joint(0).dof() == 0)
  {
    solver().setContacts({});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("MCBody6dController does not support robot {}", robot().name());
  }

  std::string body = robot().mb().bodies().back().name();
  if(robot().hasBody("RARM_LINK7"))
  {
    body = "RARM_LINK7";
    efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  else if(robot().hasBody("r_wrist"))
  {
    body = "r_wrist";
    efTask.reset(new mc_tasks::EndEffectorTask("r_wrist", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  efTask = std::make_shared<mc_tasks::EndEffectorTask>(body, robots(), robots().robotIndex(), 2.0, 1e5);
  solver().addTask(efTask);
  if(robot().mb().joint(0).dof() != 0)
  {
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex());
    solver().addTask(comTask);
  }

  mc_rtc::log::success("MCBody6dController init done");
}

void MCBody6dController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  efTask->reset();
  if(comTask)
  {
    comTask->reset();
  }
}

} // namespace mc_control
