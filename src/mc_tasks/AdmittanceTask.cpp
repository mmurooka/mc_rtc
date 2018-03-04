#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace
{

void clampAndWarn(Eigen::Vector3d & vector, const Eigen::Vector3d & bound, const std::string & label)
{
  const char dirName[] = {'x', 'y', 'z'};
  for (unsigned i = 0; i < 3; i++)
  {
    if (vector(i) < -bound(i))
    {
      LOG_WARNING("AdmittanceTask: " << label << " hit lower bound along " << dirName[i] << "-coordinate");
      vector(i) = -bound(i);
    }
    else if (vector(i) > bound(i))
    {
      LOG_WARNING("AdmittanceTask: " << label << " hit upper bound along " << dirName[i] << "-coordinate");
      vector(i) = bound(i);
    }
  }
}

}

AdmittanceTask::AdmittanceTask(const std::string & surfaceName,
      const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      double timestep,
      double stiffness, double weight)
  : SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight), 
    robot_(robots.robots()[robotIndex]),
    surface_(robots.robot(robotIndex).surface(surfaceName)),
    sensor_(robot_.bodyForceSensor(surface_.bodyName())),
    X_fsactual_surf_(surface_.X_b_s() * sensor_.X_fsactual_parent()),
    timestep_(timestep)
{
  name_ = "admittance_" + robot_.name() + "_" + surfaceName;
}

void AdmittanceTask::update()
{
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampAndWarn(name_, linearVel, maxLinearVel_, "linear velocity", isClampingLinearVel_);
  clampAndWarn(name_, angularVel, maxAngularVel_, "angular velocity", isClampingAngularVel_);
  refVel_ = sva::MotionVecd(angularVel, linearVel);

  // SC: we could do add an anti-windup strategy here, e.g. back-calculation.
  // Yet, keep in mind that our velocity bounds are artificial. Whenever
  // possible, the best is to set to gains so that they are not saturated.

  this->refVel(refVel_);
}

void AdmittanceTask::reset()
{
  SurfaceTransformTask::reset();
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  refVel_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_admittance",
                     [this]() -> const sva::ForceVecd &
                     {
                     return admittance_;
                     });
  logger.addLogEntry(name_ + "_measured_wrench",
                     [this]() -> sva::ForceVecd
                     {
                     return measuredWrench();
                     });
  logger.addLogEntry(name_ + "_ref_vel",
                     [this]() -> sva::MotionVecd
                     {
                     return refVel_;
                     });
  logger.addLogEntry(name_ + "_target_wrench",
                     [this]() -> const sva::ForceVecd &
                     {
                     return targetWrench_;
                     });
  logger.addLogEntry(name_ + "_world_measured_wrench",
                     [this]() -> sva::ForceVecd
                     {
                     return worldMeasuredWrench();
                     });
}

void AdmittanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_wrench");
  logger.removeLogEntry(name_ + "_ref_vel");
  logger.removeLogEntry(name_ + "_target_wrench");
  logger.removeLogEntry(name_ + "_world_measured_wrench");
}

} // mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("admittance",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::AdmittanceTask>(config("surface"), solver.robots(), config("robotIndex"), solver.dt());
    if(config.has("admittance")) { t->admittance(config("admittance")); }
    if(config.has("damping")) { t->damping(config("damping")); }
    if(config.has("pose")) { t->targetPose(config("pose")); }
    if(config.has("weight")) { t->weight(config("weight")); }
    if(config.has("wrench")) { t->targetWrench(config("wrench")); }
    t->load(solver, config);
    return t;
  }
);

}
