#include "Go1.h"
#include "config.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

Go1::Go1() : mc_rbdyn::RobotModule(mc_rtc::GO1_DESCRIPTION_PATH, "go1")
{
  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));
  rsdf_dir = path + "/rsdf/" + name + "/";
  // Automatically load the convex hulls associated to each body
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
  mc_rtc::log::success("PandaRobotModule uses urdf_path {}", urdf_path);
  mc_rtc::log::success("PandaRobotModule uses convexPath {}", convexPath);
  mc_rtc::log::success("PandaRobotModule uses rsdf_dir {}", rsdf_dir);
  // Ref joint order
  _ref_joint_order = {"FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
                      "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
                      "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};
  // Stance: joint name, angle in degrees
  // // Default joint configuration, if a joint is omitted the configuration is 0 or the middle point of the limit range
  // if
  // // 0 is not a valid configuration
  std::map<std::string, double> standing{
      {"FR_hip_joint", 0.0},    {"FR_thigh_joint", 0.67}, {"FR_calf_joint", -1.3},  {"FL_hip_joint", 0.0},
      {"FL_thigh_joint", 0.67}, {"FL_calf_joint", -1.3},  {"RR_hip_joint", 0.0},    {"RR_thigh_joint", 0.67},
      {"RR_calf_joint", -1.3},  {"RL_hip_joint", 0.0},    {"RL_thigh_joint", 0.67}, {"RL_calf_joint", -1.3},
  };

  for(const auto & j : mb.joints())
  {
    if(j.name() != "Root" && j.dof() > 0) { _stance[j.name()] = {standing.at(j.name())}; }
  }
  // // Default configuration of the floating base
  _default_attitude = {1., 0., 0., 0., 0., 0., 0.35};

  _bodySensors.emplace_back("Accelerometer", "trunk", sva::PTransformd(Eigen::Vector3d(0., 0., 0.)));
  _bodySensors.emplace_back("FloatingBase", "trunk", sva::PTransformd::Identity());
  // // Define a minimal set of self-collisions
  _minimalSelfCollisions = {{"FR_hip", "trunk", 0.02, 0.01, 0.0},
                            {"FL_hip", "trunk", 0.02, 0.01, 0.0},
                            {"FR_calf", "RR_thigh", 0.02, 0.01, 0.0},
                            {"FR_calf", "RR_calf", 0.02, 0.01, 0.0},
                            {"FL_calf", "RL_thigh", 0.02, 0.01, 0.0},
                            {"FL_calf", "RL_calf", 0.02, 0.01, 0.0},
                            {"FR_calf", "FL_calf", 0.02, 0.01, 0.0},
                            {"RR_calf", "RL_calf", 0.02, 0.01, 0.0},
                            {"RR_hip", "trunk", 0.02, 0.01, 0.0},
                            {"RL_hip", "trunk", 0.02, 0.01, 0.0},
                            {"FR_thigh", "FL_thigh", 0.02, 0.01, 0.0},
                            {"FR_thigh", "FL_calf", 0.02, 0.01, 0.0},
                            {"FR_calf", "FL_thigh", 0.02, 0.01, 0.0},
                            {"RR_thigh", "RL_thigh", 0.02, 0.01, 0.0},
                            {"RR_thigh", "RL_calf", 0.02, 0.01, 0.0},
                            {"RR_calf", "RL_thigh", 0.02, 0.01, 0.0}

  };

  _commonSelfCollisions = _minimalSelfCollisions;
}

} // namespace mc_robots

#include <mc_rbdyn/RobotModuleMacros.h>

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("Go1", mc_robots::Go1)
