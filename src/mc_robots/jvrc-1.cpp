#ifndef JVRC_DESCRIPTION_PATH
#  error "JVRC_DESCRIPTION_PATH must be defined to build this RobotModule"
#endif

#define JVRC_VAL(x) #x
#define JVRC_VAL_VAL(x) JVRC_VAL(x)

#include "jvrc-1.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

JVRC1RobotModule::JVRC1RobotModule() : RobotModule(std::string(JVRC_VAL_VAL(JVRC_DESCRIPTION_PATH)), "jvrc-1")
{
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false);
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    boundsFromURDF(res.limits);
    _collisionTransforms = res.collision_tf;

    std::string convexPath = path + "/convex/" + name + "/";
    bfs::path p(convexPath);
    if(bfs::exists(p) && bfs::is_directory(p))
    {
      std::vector<bfs::path> files;
      std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
      for(const bfs::path & file : files)
      {
        size_t off = file.filename().string().rfind("-ch.txt");
        if(off != std::string::npos)
        {
          std::string name = file.filename().string();
          name.replace(off, 7, "");
          _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
        }
      }
    }
    expand_stance();
    make_default_ref_joint_order();
    std::vector<double> default_q = {-0.38, -0.01, 0., 0.72, -0.01, -0.33,  -0.38, 0.02, 0.,    0.72, -0.02, -0.33, 0.,
                                     0.13,  0.,    0., 0.,   0.,    -0.052, -0.17, 0.,   -0.52, 0.,   0.,    0.,    0.,
                                     0.,    0.,    0., 0.,   0.,    -0.052, 0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,
                                     0.,    0.,    0., 0.,   0.,    0.,     0.,    0.,   0.,    0.,   0.};
    const auto & rjo = ref_joint_order();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      _stance[rjo[i]] = {default_q[i]};
    }
    _default_attitude = {{1., 0., 0., 0., 0., 0., 0.8275}};
    _forceSensors.push_back(
        mc_rbdyn::ForceSensor("RightFootForceSensor", "R_ANKLE_P_S", sva::PTransformd::Identity()));
    _forceSensors.push_back(
        mc_rbdyn::ForceSensor("LeftFootForceSensor", "L_ANKLE_P_S", sva::PTransformd::Identity()));
    _forceSensors.push_back(
        mc_rbdyn::ForceSensor("RightHandForceSensor", "R_WRIST_Y_S", sva::PTransformd::Identity()));
    _forceSensors.push_back(
        mc_rbdyn::ForceSensor("LeftHandForceSensor", "L_WRIST_Y_S", sva::PTransformd::Identity()));

    _minimalSelfCollisions = {
      {"WAIST_R_S", "L_SHOULDER_Y_S", 0.02, 0.001, 0.},
      {"WAIST_R_S", "R_SHOULDER_Y_S", 0.02, 0.001, 0.},
      {"PELVIS_S", "R_ELBOW_P_S", 0.05, 0.001, 0.},
      {"PELVIS_S", "L_ELBOW_P_S", 0.05, 0.001, 0.},
      {"R_WRIST_Y_S", "R_HIP_Y_S", 0.05, 0.025, 0.},
      {"L_WRIST_Y_S", "L_HIP_Y_S", 0.05, 0.025, 0.}
    };
    _commonSelfCollisions = _minimalSelfCollisions;
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Could not load JVRC-1 model at " << urdf_path)
  }
}

} // namespace mc_robots
