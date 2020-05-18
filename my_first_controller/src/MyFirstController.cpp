#include "MyFirstController.h"


// Make shorter names for types we will use a lot
using Color = mc_rtc::gui::Color;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
using Range = mc_rtc::gui::plot::Range;
using Style = mc_rtc::gui::plot::Style;
using Side = mc_rtc::gui::plot::Side;

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
  : mc_control::MCController({rm,
        mc_rbdyn::RobotLoader::get_robot_module
        ("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH) + "/../mc_int_obj_description", std::string("door")),
        mc_rbdyn::RobotLoader::get_robot_module
        ("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  // solver().setContacts({{}});
  solver().setContacts({
      {robots(), 0, 2, "LeftFoot", "AllGround"},
        {robots(), 0, 2, "RightFoot", "AllGround"}
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
  // if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05)
  //   {
  //     switch_target();
  //   }

  // if(comTask->eval().norm() < 0.01)
  //   {
  //     switch_com_target();
  //   }

  switch_phase();

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

  // In the reset function
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.7, 0.5, 0)));

  // In the reset function
  doorKinematics = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
  solver().addConstraintSet(*doorKinematics);
  doorPosture = std::make_shared<mc_tasks::PostureTask>(solver(), 1, 5.0, 1000.0);
  solver().addTask(doorPosture);
  doorPosture->reset();

  // In the reset function
  // Create the task and add it to the solver
  handTask = std::make_shared<mc_tasks::SurfaceTransformTask>("RightGripper", robots(), 0, 5.0, 1000.0);
  solver().addTask(handTask);
  // Set a target relative to the handle position
  handTask->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.025)) * robots().robot(1).surfacePose("Handle"));

  gui()->addElement({"gui_test", "button"},
                    mc_rtc::gui::Button("Push", []() { std::cout << "Hello!" << std::endl; }));
  gui()->addElement({"gui_test", "label"},
                    mc_rtc::gui::Label("LabelText", [this]() { return std::to_string(this->phase); }));

  gui()->addPlot("sin(t)",
                 mc_rtc::gui::plot::X("t",
                                      [this]() { return this->logger().t(); }),
                 mc_rtc::gui::plot::Y("sin(t)",
                                      [this]() { return sin(this->logger().t()); },
                                      Color::Red)
                 );
  gui()->addXYPlot("Circle in a square",
                   mc_rtc::gui::plot::XY("Circle",
                                         [this]() { return cos(this->logger().t()); },
                                         [this]() { return sin(this->logger().t()); },
                                         Color::Red),
                   mc_rtc::gui::plot::Polygon("Square",
                                              []() { return PolygonDescription({{-1, 1}, {-1, 1}, {1, 1}, {1, -1}}, Color::Blue); })
                   );
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

// A new method for our controller
void MyFirstController::switch_phase()
{
  if(phase == APPROACH &&
     handTask->eval().norm() < 0.05 &&
     handTask->speed().norm() < 1e-4)
    {
      // Add a new contact
      auto contacts = solver().contacts();
      contacts.emplace_back(robots(), 0, 1, "RightGripper", "Handle");
      solver().setContacts(contacts);
      // Remove the surface transform task
      solver().removeTask(handTask);
      // Keep the robot in its current posture
      postureTask->reset();
      comTask->reset();
      // Target new handle position
      doorPosture->target({{"handle", {-1.0}}});
      // Switch phase
      phase = HANDLE;
    }
  else if(phase == HANDLE &&
          doorPosture->eval().norm() < 0.01)
    {
      // Update door opening target
      doorPosture->target({{"door", {0.5}}});
      // Switch phase
      phase = OPEN;
    }
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
