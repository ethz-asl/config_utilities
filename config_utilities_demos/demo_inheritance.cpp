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
  float e = 0.f;
  int f = 0;

  void fromRosParam() override {
    rosParam("e", &e);
    rosParam("f", &f);
  }
  void printFields() const override {
    printField("e", e);
    printField("f", f);
  }
};

// Define a base class that uses a base config.
class MyBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int a = 0;
    double b = 0.0;
    OtherConfig other_cfg;

    void fromRosParam() override {
      rosParam("a", &a);
      rosParam("b", &b);
      //rosParam("other_cfg", &other_cfg);
    }
    void printFields() const override {
      printField("a", a);
      printField("b", b);
      printField("other_cfg", other_cfg);
    }
  };

  explicit MyBase(const Config& config) : config_(config.checkValid()) {}
  virtual ~MyBase() = default;

 private:
  const Config config_;
};

// Define a derived class that uses its own config.
class MyDerived : public MyBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string c = "c";
    bool d = false;

    // Configs are allowed to contain other configs, so we add the base and
    // another config here.
    MyBase::Config base_config;
    OtherConfig other_config;

    void fromRosParam() override {
      rosParam("c", &c);
      rosParam("d", &d);

      // The usual rosParam interfaces also work on configs. These don't
      // require a param name and operate on the same namesapce.
//      rosParam(&base_config);
//      rosParam(&other_config);
    }
    void printFields() const override {
      printField("c", c);
      printField("d", d);

      // The usual printField interfaces also work on configs;
      printField("base_config", base_config);
      printField("other_config", other_config);
    }
    Config() {
      setName("MyDerivedConfig");
      setPrintWidth(60);
      setPrintIndent(20);
    }
  };

  // We can use the member config to initialize the base class.
  explicit MyDerived(const Config& config)
      : MyBase(config.base_config), config_(config.checkValid()) {}

  void print() const {
    std::cout << config_.toString() << std::endl;
  }

 private:
  const Config config_;
};

int main(int argc, char** argv) {
  // Warnings are printed using GLOG, make sure to run with "-alsologtostderr".
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server
  ros::init(argc, argv, "demo_inheritance");
  ros::NodeHandle nh_private("~");
  nh_private.setParam("a", 1);
  nh_private.setParam("b", 2.0);
  nh_private.setParam("c", "three");
  nh_private.setParam("d", true);
  nh_private.setParam("e", 4.0);
  nh_private.setParam("f", 5);


  // We can as usual get the derived config and instantiate a derived object.
  MyDerived::Config config; // =
//      config_utilities::getConfigFromRos<MyDerived::Config>(nh_private);
  MyDerived derived(config);
  derived.print();

  return 0;
}
