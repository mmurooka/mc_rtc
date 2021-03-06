/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include "benchmark/benchmark.h"

class RobotLoadingFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State &)
  {
    // XXX silence error output
    std::cerr.rdbuf(nullptr);

    mc_rbdyn::RobotLoader::clear();
    mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  }

  void TearDown(const ::benchmark::State &) {}
};

BENCHMARK_DEFINE_F(RobotLoadingFixture, RobotModuleLoading)(benchmark::State & state)
{
  while(state.KeepRunning())
  {
    auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  }
}
BENCHMARK_REGISTER_F(RobotLoadingFixture, RobotModuleLoading)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(RobotLoadingFixture, RobotCreation)(benchmark::State & state)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  while(state.KeepRunning())
  {
    auto robots = mc_rbdyn::loadRobot(*rm);
  }
}
BENCHMARK_REGISTER_F(RobotLoadingFixture, RobotCreation)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(RobotLoadingFixture, RobotCopy)(benchmark::State & state)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots_ptr = mc_rbdyn::loadRobot(*rm);
  const auto & robots = *robots_ptr;
  while(state.KeepRunning())
  {
    auto robots_copy = robots;
  }
}
BENCHMARK_REGISTER_F(RobotLoadingFixture, RobotCopy)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
