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

// NOTE: If this is included before config_utilities, a specialization for
// transformations from ros params is enabled. Works outside of ROS.
#include <kindr/minimal/quat-transformation.h>

#include "../config_utilities.hpp"

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

// Define a config struct.
struct Config : public config_utilities::Config<Config> {
  int a = 0;
  double b = 0;
  std::string c = "default";
  std::vector<double> vec;
  std::map<std::string, int> map;
  Transformation T;
  std::string ns;

  // Optional other fields can be set in the constructor.
  Config() {
    setConfigName("Config (from ROS params)");
    setPrintWidth(60);
    setPrintIndent(15);
  }

 protected:
  // Define the parameter names to read from ROS param server.
  void fromRosParam() override {
    rosParam("a", &a);
    rosParam("b", &b);
    rosParam("c", &c);
    rosParam("vec", &vec);
    rosParam("map", &map);
    rosParam("T", &T);  // This is enabled due to minkindr include.

    // Any custom data can be gathered via XmlRpc.
    XmlRpc::XmlRpcValue xml_rpc_value;
    rosParam("custom_data", &xml_rpc_value);
    /* Do some magic with 'xml_rpc_value' here */

    // The namespace of the nodehandle is exposed via rosParamNameSpace():
    ns = rosParamNameSpace();
  }

  // Optionally implement printing.
  void printFields() const override {
    printField("a", a);
    printField("b", b);
    printField("c", c);
    printField("vec", vec);
    printField("map", map);
    printField("T", T);  // This is enabled due to minkindr include.
    printField("namespace", ns);
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
