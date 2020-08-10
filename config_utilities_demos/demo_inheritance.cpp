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

#include "../config_utilities.hpp"

// The method introduced below also works for other configs, so we declare
// another config.
struct OtherConfig : public config_utilities::Config<OtherConfig> {
  float a = 0.f;
  int b = 0;

  void fromRosParam() override {
    rosParam("a", &a);
    rosParam("b", &b);
  }
  void printFields() const override {
    printField("a", a);
    printField("b", b);
  }
};

// Define a base class that uses a base config.
class MyBase {
public:
  struct Config : public config_utilities::Config<Config> {
    bool c = true;
    double d = 0.0;

    // Configs are allowed to contain other configs.
    OtherConfig other_config;

    void fromRosParam() override {
      rosParam("c", &c);
      rosParam("d", &d);

      // The usual rosParam interfaces also work on configs. These don't require
      // a param name, but optionally a sub_namespace can be specified.
      rosParam(&other_config);
    }
    void printFields() const override {
      printField("c", c);
      printField("d", d);

      // The usual printFields() method also works on configs.
      printField("other_config", other_config);
    }
  };

  explicit MyBase(const Config &config) : config_(config.checkValid()) {}
  virtual ~MyBase() = default;

private:
  const Config config_;
};

// Define a derived class that uses its own config.
class MyDerived : public MyBase {
public:
  struct Config : public config_utilities::Config<Config> {
    std::string e = "e";
    int f = 0;

    // Configs are allowed to contain other configs, so we add the base config.
    MyBase::Config base_config;
    OtherConfig other_config;

    void fromRosParam() override {
      rosParam("e", &e);
      rosParam("f", &f);

      // Here we use a sub_namespace 'base' to create the base config from.
      rosParam(&base_config, "base");
      rosParam(&other_config);
    }
    void printFields() const override {
      printField("e", e);
      printField("f", f);
      printField("other_config", other_config);
      printField("base_config", base_config);
    }
    Config() {
      setName("MyDerivedConfig");
      setPrintWidth(60);
      setPrintIndent(20);
    }
  };

  // We can use the member config to initialize the base class.
  explicit MyDerived(const Config &config)
      : MyBase(config.base_config), config_(config.checkValid()) {}

  void print() const { std::cout << config_.toString() << std::endl; }

private:
  const Config config_;
};

int main(int argc, char **argv) {
  // Warnings are printed using GLOG, make sure to run with "-alsologtostderr".
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server
  ros::init(argc, argv, "demo_inheritance");
  ros::NodeHandle nh_private("~");
  nh_private.setParam("base/a", 1.0);
  nh_private.setParam("base/b", 2);
  nh_private.setParam("base/c", false);
  nh_private.setParam("base/d", 3.45);
  nh_private.setParam("a", -11.1);
  nh_private.setParam("b", 222);
  nh_private.setParam("e", "Bananas are yellow.");
  nh_private.setParam("f", 6.78);

  // We can as usual get the derived config and instantiate a derived object.
  MyDerived::Config config =
      config_utilities::getConfigFromRos<MyDerived::Config>(nh_private);
  MyDerived derived(config);
  derived.print();

  return 0;
}