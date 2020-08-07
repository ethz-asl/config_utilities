/**
 * This demo showcases the use of config_utilities create a Config from ros
 * parameter server. This functionality is only included if ros/node_handle.h
 * is included, i.e. in a ROS library.
 */

#include <iostream>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ros-tools
// (i.e. getConfigFromRos()) are enabled.
#include <ros/node_handle.h>

// NOTE: If this is included before config_utilities, a specialization for
// transformations from ros params is enabled. Works outside of ROS.
#include <kindr/minimal/quat-transformation.h>

// NOTE: If this is included before config_utilities, a specialization for
// XmlRpcValues is from ros params is enabled. Works outside of ROS.
#include <xmlrpcpp/XmlRpcValue.h>


#include "../config_utilities.hpp"

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

// Define a config struct.
struct Config : public config_utilities::Config<Config> {
  int a = 0;
  double b = 0;
  std::string c = "default";
  std::string d = "default";
  Transformation t;

  // Define the parameter names to read from ROS param server.
  void fromRosParam() override {
    rosParam("a", &a);
    rosParam("b", &b);
    rosParam("c", &c);
    rosParam("d", &d);
    rosParam("t", &t);
  }

  // Optionally implement printing.
  void printFields() const override {
    printField("a", a);
    printField("b", b);
    printField("c", c);
    printField("d", d);
    std::stringstream ss;
    ss << t.asVector().transpose();
    printField("t", ss.str());
  }

  // Optional other fields can be set in the constructor.
  Config() {
    setName("Config (from ROS params)");
    setPrintWidth(40);
    setPrintIndent(20);
  }
};


int main(int argc, char** argv) {
  // Warnings are printed using GLOG, make sure to run with "-alsologtostderr".
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server
  ros::init(argc, argv, "demo_ros_param");
  ros::NodeHandle nh_private("~");
  nh_private.setParam("a", 123);
  nh_private.setParam("b", 45.6);
  nh_private.setParam("c", "seven-eight-nine");


  // Getting the config from ros params is a one liner.
  Config config = config_utilities::getConfigFromRos<Config>(nh_private);

  // Verify it worked.
  std::cout << " ----- ROS Parameter server ----- " << std::endl;
  std::cout << "a: " << nh_private.param("a", 0) << std::endl;
  std::cout << "b: " << nh_private.param("b", 0.0) << std::endl;
  std::cout << "c: " << nh_private.param("c", std::string("")) << std::endl;

  std::cout << config.toString() << std::endl;

  return 0;
}
