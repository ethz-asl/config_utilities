/**
 * This demo showcases the use of config_utilities create a Config from ros
 * parameter server. This functionality is only included if ros/node_handle.h
 * is included, i.e. in a ROS library.
 */

#include <iostream>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ROS-tools,
// i.e. getConfigFromRos(), are enabled.
#include <ros/node_handle.h>


#include "../config_utilities.hpp"

// Define a config struct.
struct Config : public config_utilities::Config<Config> {
  int a = 0;
  double b = 0;
  std::string c = "default";
  std::vector<double> vec;
  std::map<std::string, int> map;
  std::string ns;

  // Define the parameter names to read from ROS param server.
  void setupParamsAndPrinting() override {
    setupParam("a", &a);
    setupParam("b", &b);
    setupParam("c", &c);
    setupParam("vec", &vec);
    setupParam("map", &map);
  }

  // Optional other fields can be set in the constructor.
  Config() {
    setConfigName("Config (from ROS params)");
    setPrintWidth(60);
    setPrintIndent(15);
  }
};

int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server.
  ros::init(argc, argv, "demo_ros_param");
  ros::NodeHandle nh_private("~");
  std::vector<double> vec{1.0, 2.0, 3.0};
  std::map<std::string, int> map;
  map["m1"] = 1;
  map["m2"] = 2;
  nh_private.setParam("a", 123);
  nh_private.setParam("b", 45.6);
  nh_private.setParam("c", "seven-eight-nine");
  nh_private.setParam("vec", vec);
  nh_private.setParam("map", map);

  // Getting the config from ros params is a one liner.
  Config config = config_utilities::getConfigFromRos<Config>(nh_private);

  // Verify it worked.
  std::cout << config.toString() << std::endl;

  return 0;
}
