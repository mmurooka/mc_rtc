#include "MyFirstController.h"

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  // solver().setContacts({{}});
  solver().setContacts({
      {robots(), 0, 1, "LeftFoot", "AllGround"},
        {robots(), 0, 1, "RightFoot", "AllGround"}
    });

  jointIndex = robot().jointIndexByName("NECK_Y");

  // In the constructor, create the task and add it to the problem
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);
  // Reduce the posture task stiffness
  postureTask->stiffness(1);

  // In the constructor, create the task and add it to the problem
  efTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
  solver().addTask(efTask);

  // Get the task from the JSON/YAML file (tried acoording to tutorial but caused SEGV)
  // std::string task_file_path("/home/mmurooka/jrl/mc_rtc/my_first_controller/config/task.json");
  // auto task = mc_tasks::MetaTaskLoader::load(solver(), task_file_path);
  // solver().addTask(task);

  LOG_SUCCESS("MyFirstController init done " << this)
}

bool MyFirstController::run()
{
  if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05)
    {
      switch_target();
    }

  if(comTask->eval().norm() < 0.01)
    {
      switch_com_target();
    }

  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  // In the reset function, reset the task to the current CoM
  comTask->reset();
  comZero = comTask->com();

  // In the reset function, reset the task to the current EF position
  efTask->reset();
}

void MyFirstController::switch_target()
{
  if(goingLeft)
    {
      postureTask->target({{"NECK_Y", robot().qu()[jointIndex]}});
    }
  else
    {
      postureTask->target({{"NECK_Y", robot().ql()[jointIndex]}});
    }
  goingLeft = !goingLeft;
}

void MyFirstController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if(comDown)
    {
      comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});
    }
  else
    {
      comTask->com(comZero);
    }
  comDown = !comDown;
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
