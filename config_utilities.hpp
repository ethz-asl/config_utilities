/**
AUTHOR: Lukas Schmid <schmluk@mavt.ethz.ch>

Copyright 2020 Autonomous Systems Lab (ASL), ETH Zürich.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Depending on which headers are available, ROS dependencies are included in
 * the library. Make sure to include config_utilities.hpp after these headers.
 */

#include <ros/node_handle.h>
#include <kindr/minimal/quat-transformation.h>

// <ros/node_handle.h>
#ifdef ROSCPP_NODE_HANDLE_H
#define CONFIG_UTILITIES_ROS_ENABLED
#endif

// <kindr/minimal/quat-transformation.h>
#ifdef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#define CONFIG_UTILITIES_TRANSFORMATION_ENABLED
#endif

#ifndef CONFIG_UTILITIES_CORE_HPP_
#define CONFIG_UTILITIES_CORE_HPP_

#include <string>
#include <utility>
#include <memory>
#include <typeinfo>
#include <type_traits>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <map>

#include <glog/logging.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace config_utilities {

/**
 * ==================== Defaults ====================
 */
namespace internal {
struct Settings {
  // Settings
  unsigned int default_print_width = 80;
  unsigned int default_print_indent = 30;
  unsigned int default_subconfig_indent = 3;

  static Settings& instance() {
    static Settings settings;
    return settings;
  }
 private:
  Settings() = default;
};
} // namespace internal

// Access.
internal::Settings& Settings() { return internal::Settings::instance(); }

/**
 * ==================== Internal Utilities ====================
 */
namespace internal {
// Printing tool.
std::string printCenter(const std::string& text, int width, char symbol = ' ');

// Type verification.
template<typename T>
bool isConfig(const T* candidate);

// Base class for internal use.
struct ConfigBase {
  [[nodiscard]] virtual std::string toStringInternal(int indent,
                                                     int print_width,
                                                     int print_indent) const = 0;
};
// This is a dummy operator, configs provide toString().
std::ostream& operator<<(std::ostream& os, const ConfigBase&) { return os; }

// Creation interface.
using ParamMap = std::unordered_map<std::string, XmlRpc::XmlRpcValue>;
template<typename ConfigType>
void setupConfigFromParamMap(const ParamMap& params, ConfigType* config);
} // namespace internal

/**
 * ==================== ConfigChecker Interface ====================
 */

// Utility tool to make checking configs easier and more readable.
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name);
  ~ConfigChecker() = default;

  // Get the result of all checks.
  [[nodiscard]] bool isValid(bool print_warnings = false) const;
  void checkValid() const;

  // Reset checks.
  void reset();

  // Checking interface.
  template<typename T>
  void checkGT(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkGE(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkLT(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkLE(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkEq(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkNE(const T& param, const T& value, const std::string& name);
  void checkCond(bool condition, const std::string& warning);

  // Printing interfaces.
  void setPrintWidth(int width);

 private:
  void print(const std::string& severity) const;

 private:
  const std::string name_;
  std::vector<std::string> warnings_;
  int print_width_;
};

/**
 * ==================== Config Interface ====================
 */
template<typename ConfigT>
struct Config : public internal::ConfigBase {
 public:
  // Construction.
  Config();
  Config(const Config& other);
  Config& operator=(const Config& other);

  // Validity Checking.
  [[nodiscard]] bool isValid(bool print_warnings = false) const;
  ConfigT checkValid() const;
  ConfigT& checkValid();

  // Implementable setup tool.
  virtual void initializeDependentVariableDefaults() {}

  // Printing tool
  [[nodiscard]] std::string toString() const;

 protected:
  // Implementable methods.
  virtual void checkParams() const {};
  virtual void printFields() const;
  virtual void fromRosParam();

  // General Tools.
  void setName(const std::string& name) { name_ = name; }
  void setPrintWidth(int width) { meta_data_->print_width = width; }
  void setPrintIndent(int indent) { meta_data_->print_indent = indent; }

  // Checker Tools.
  template<typename T>
  void checkParamGT(const T& param,
                    const T& value,
                    const std::string& name) const;
  template<typename T>
  void checkParamGE(const T& param,
                    const T& value,
                    const std::string& name) const;
  template<typename T>
  void checkParamLT(const T& param,
                    const T& value,
                    const std::string& name) const;
  template<typename T>
  void checkParamLE(const T& param,
                    const T& value,
                    const std::string& name) const;
  template<typename T>
  void checkParamEq(const T& param,
                    const T& value,
                    const std::string& name) const;
  template<typename T>
  void checkParamNE(const T& param,
                    const T& value,
                    const std::string& name) const;
  void checkParamCond(bool condition, const std::string& warning) const;

  // Printing Tools.
  template<typename T>
  void printField(const std::string& name, const T& field) const;
  void printText(const std::string& text) const;
  // Specialization for vectors and maps.
  template<typename T>
  void printField(const std::string& name, const std::vector<T>& field) const;
  template<typename T>
  void printField(const std::string& name,
                  const std::map<std::string, T>& field) const;

  // Param Tools. These are explicitely overloaded to assure compatibility with
  // ROS and warn the user about compatibility
  void rosParam(const std::string& name, int* param);
  void rosParam(const std::string& name, float* param);
  void rosParam(const std::string& name, double* param);
  void rosParam(const std::string& name, bool* param);
  void rosParam(const std::string& name, std::string* param);
  void rosParam(const std::string& name, std::vector<int>* param);
  void rosParam(const std::string& name, std::vector<double>* param);
  void rosParam(const std::string& name, std::vector<float>* param);
  void rosParam(const std::string& name, std::vector<bool>* param);
  void rosParam(const std::string& name, std::vector<std::string>* param);
  void rosParam(const std::string& name, std::map<std::string, int>* param);
  void rosParam(const std::string& name, std::map<std::string, double>* param);
  void rosParam(const std::string& name, std::map<std::string, float>* param);
  void rosParam(const std::string& name, std::map<std::string, bool>* param);
  void rosParam(const std::string& name,
                std::map<std::string, std::string>* param);
  void rosParam(const std::string& name, XmlRpc::XmlRpcValue* param);
  template<typename ConfigType>
  void rosParam(Config<ConfigType>* param, const std::string& sub_namespace="");


    // Additional specialization for transformations if they are included.
#ifdef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
  template<typename Scalar>
  void printField(const std::string& name,
                  const kindr::minimal::QuatTransformationTemplate<Scalar>& field) const;
  template<typename Scalar>
  void rosParam(const std::string& name,
                kindr::minimal::QuatTransformationTemplate<Scalar>* param);
#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED

 private:
  template<typename ConfigType> friend void internal::setupConfigFromParamMap(
      const internal::ParamMap& params, ConfigType* config);

  struct MetaData {
    std::unique_ptr<ConfigChecker> checker;
    std::unique_ptr<std::vector<std::string>> messages;
    const internal::ParamMap* params = nullptr;
    int print_width = Settings().default_print_width;
    int print_indent = Settings().default_print_indent;
    int indent = 0;  // Only used for nested printing.

    MetaData() = default;
    MetaData(const MetaData& other) {
      print_width = other.print_width;
      print_indent = other.print_indent;
      indent = other.indent;
    }
  };

  // Implementation for printing fields.
  template<typename T>
  void printFieldInternal(const std::string& name, const T& field) const;
  // Specialized function to print configs.
  void printConfigInternal(const std::string& name,
                           const internal::ConfigBase* field) const;

  [[nodiscard]] std::string toStringInternal(int indent,
                                             int print_width,
                                             int print_indent) const override;

  // Ros params and setup
  void setupFromParmaMap(const internal::ParamMap& params);

  template<typename T>
  void rosParamInternal(const std::string& name, T* param);

  template<typename T>
  void rosParamMapInternal(const std::string& name,
                                            std::map<std::string,T>* param);

 private:
  std::string name_;
  std::unique_ptr<MetaData> meta_data_;
};

/**
 * ==================== ConfigChecker Implementation ====================
 */

ConfigChecker::ConfigChecker(std::string module_name) :
    name_(std::move(module_name)),
    print_width_(Settings().default_print_width) {}

bool ConfigChecker::isValid(bool print_warnings) const {
  if (warnings_.empty()) {
    return true;
  }
  if (print_warnings) {
    print("Warning");
  }
  return false;
}

void ConfigChecker::checkValid() const {
  if (!isValid(false)) {
    print("Error");
    const bool config_params_are_valid = false;
    CHECK(config_params_are_valid);
  }
}

void ConfigChecker::reset() { warnings_.clear(); }

template<typename T>
void ConfigChecker::checkGT(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param <= value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected > '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkGE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param < value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected >= '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkLT(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param >= value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected < '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkLE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param > value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected <= '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkEq(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param != value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected to be '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkNE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param == value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected to be different from '" << value
       << "'.";
    warnings_.emplace_back(ss.str());
  }
}

void ConfigChecker::checkCond(bool condition, const std::string& warning) {
  if (!condition) {
    warnings_.emplace_back(warning);
  }
}

void ConfigChecker::setPrintWidth(int width) { print_width_ = width; }

void ConfigChecker::print(const std::string& severity) const {
  std::string sev = "";
  if (!severity.empty()) {
    sev.append(severity + ": ");
  }
  int length = print_width_ - sev.length();
  std::string warning = "\n" + internal::printCenter(name_, print_width_, '=');
  for (std::string w : warnings_) {
    std::string line = sev;
    while (w.length() > length) {
      line.append(w.substr(0, length));
      w = w.substr(length);
      warning.append("\n" + line);
      line = std::string(sev.length(), ' ');
    }
    warning.append("\n" + line + w);
  }
  warning = warning + "\n" + std::string(print_width_, '=');
  LOG(ERROR) << warning;
}

/**
 * ==================== Utilities Implementation ====================
 */
namespace internal {
std::string printCenter(const std::string& text, int width, char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width -
      static_cast<int>(result.length()), 0), symbol);
  return result;
}

template<typename T>
bool isConfig(const T* candidate) {
  try { throw candidate; }
  catch (const ConfigBase*) {
    return true;
  }
  catch (...) {
  }
  return false;
}

template<typename ConfigType>
void setupConfigFromParamMap(const ParamMap& params, ConfigType* config) {
// This assumes that ConfigT is a config.
  auto cfg = dynamic_cast<Config<ConfigType>*>(config);
  cfg->setupFromParmaMap(params);
}

// XML casts
bool xmlCast(const XmlRpc::XmlRpcValue& xml, bool* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean : {
      if (param) { *param = static_cast<bool>(xml); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt : {
      if (param) { *param = static_cast<bool>(static_cast<int>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble : {
      if (param) { *param = static_cast<bool>(static_cast<double>(xml)); }
      return true;
    }
    default : return false;
  }
}

bool xmlCast(const XmlRpc::XmlRpcValue& xml, int* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean : {
      if (param) { *param = static_cast<int>(static_cast<bool>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt : {
      if (param) { *param = static_cast<int>(xml); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble : {
      if (param) { *param = static_cast<int>(static_cast<double>(xml)); }
      return true;
    }
    default: return false;
  }
}

bool xmlCast(const XmlRpc::XmlRpcValue& xml, float* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean : {
      if (param) { *param = static_cast<float>(static_cast<bool>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt : {
      if (param) { *param = static_cast<float>(static_cast<int>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble : {
      if (param) { *param = static_cast<float>(static_cast<double>(xml)); }
      return true;
    }
    default: return false;
  }
}

bool xmlCast(const XmlRpc::XmlRpcValue& xml, double* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean : {
      if (param) { *param = static_cast<double>(static_cast<bool>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt : {
      if (param) { *param = static_cast<double>(static_cast<int>(xml)); }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble : {
      if (param) { *param = static_cast<double>(xml); }
      return true;
    }
    default: return false;
  }
}

bool xmlCast(const XmlRpc::XmlRpcValue& xml, std::string* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeString : {
      if (param) { *param = static_cast<std::string>(xml); }
      return true;
    }
    default: return false;
  }
}

template<typename T>
bool xmlCast(const XmlRpc::XmlRpcValue& xml, std::vector<T>* param = nullptr) {
  if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  if (param) {
    param->resize(xml.size());
    for (int i = 0; i < xml.size(); i++) {
      if (!internal::xmlCast(xml[i], &(param->at(i)))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace internal

/**
 * ==================== Config Implementation ====================
 */
template<typename ConfigT>
Config<ConfigT>::Config() : name_(typeid(ConfigT).name()),
                            meta_data_(new MetaData()) {}

template<typename ConfigT>
Config<ConfigT>::Config(const Config& other) : name_(other.name_),
                                               meta_data_(new MetaData(*(other.meta_data_))) {}

template<typename ConfigT>
Config<ConfigT>& Config<ConfigT>::operator=(const Config& other) {
  name_ = other.name_;
  meta_data_.reset(new MetaData(*other.meta_data_));
  return *this;
}

template<typename ConfigT>
bool Config<ConfigT>::isValid(bool print_warnings) const {
  meta_data_->checker = std::make_unique<ConfigChecker>(name_);
  meta_data_->checker->setPrintWidth(meta_data_->print_width);
  checkParams();
  bool result = meta_data_->checker->isValid(print_warnings);
  meta_data_->checker.reset(nullptr);
  return result;
}

template<typename ConfigT>
ConfigT Config<ConfigT>::checkValid() const {
  // Returns a copy of the config in the const case.
  ConfigT result(*static_cast<const ConfigT*>(this));
  result.initializeDependentVariableDefaults();
  result.meta_data_->checker = std::make_unique<ConfigChecker>(name_);
  result.meta_data_->checker->setPrintWidth(meta_data_->print_width);
  result.checkParams();
  result.meta_data_->checker->checkValid();
  result.meta_data_->checker.reset(nullptr);
  return result;
}

template<typename ConfigT>
ConfigT& Config<ConfigT>::checkValid() {
  // Returns a mutable reference.
  initializeDependentVariableDefaults();
  meta_data_->checker = std::make_unique<ConfigChecker>(name_);
  meta_data_->checker->setPrintWidth(meta_data_->print_width);
  checkParams();
  meta_data_->checker->checkValid();
  meta_data_->checker.reset(nullptr);
  return *static_cast<ConfigT*>(this);
}

template<typename ConfigT>
std::string Config<ConfigT>::toString() const {
  meta_data_->messages.reset(new std::vector<std::string>());
  printFields();
  std::string
      result = internal::printCenter(name_, meta_data_->print_width, '=');
  for (const std::string& msg : *(meta_data_->messages)) {
    result.append("\n" + msg);
  }
  result.append("\n" + std::string(meta_data_->print_width, '='));
  meta_data_->messages.reset(nullptr);
  return result;
};

template<typename ConfigT>
void Config<ConfigT>::printFields() const {
  meta_data_->messages->emplace_back(
      "\nThe 'printFields()' method is not implemented.");
}

template<typename ConfigT>
void Config<ConfigT>::fromRosParam() {
  LOG(WARNING) << "fromRosParam() is not implemented for '" << name_
               << "', no parameters will be loaded.";
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamGT(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamGT()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkGT(param, value, name);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamGE(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamGE()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkGE(param, value, name);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamLT(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamLT()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkLT(param, value, name);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamLE(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamLE()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkLE(param, value, name);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamEq(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamEq()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkEq(param, value, name);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::checkParamNE(const T& param,
                                   const T& value,
                                   const std::string& name) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamNE()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkNE(param, value, name);
}

template<typename ConfigT>
void Config<ConfigT>::checkParamCond(bool condition,
                                     const std::string& warning) const {
  if (!meta_data_->checker) {
    LOG(WARNING) << "'checkParamCond()' calls are only allowed within the "
                    "'checkParams()' method, no checks will be performed.";
    return;
  }
  meta_data_->checker->checkCond(condition, warning);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::printField(const std::string& name,
                                 const T& field) const {
  if (!meta_data_->messages) {
    LOG(WARNING) << "'printField()' calls are only allowed within the "
                    "'printFields()' method.";
    return;
  }
  if (internal::isConfig(&field)) {
    printConfigInternal(name, (const internal::ConfigBase*) &field);
  } else {
    printFieldInternal(name, field);
  }
}

#ifdef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
template<typename ConfigT>
template<typename Scalar>
void Config<ConfigT>::printField(const std::string& name,
                                 const kindr::minimal::QuatTransformationTemplate<Scalar>& field) const {
  if (!meta_data_->messages) {
    LOG(WARNING) << "'printField()' calls are only allowed within the "
                    "'printFields()' method.";
    return;
  }
  std::stringstream ss;
  auto rot = field.getEigenQuaternion().toRotationMatrix().eulerAngles(0, 1, 2);
  ss << "x: " << field.getPosition()[0]<<", y: "
  << field.getPosition()[1] <<", z: "
  << field.getPosition()[2]<<", roll: "
  << rot.x() <<", pitch: " << rot.y()<<", yaw: " << rot.z();
  printFieldInternal(name, ss.str());
}
#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::printField(const std::string& name,
                                 const std::vector<T>& field) const {
  if (!meta_data_->messages) {
    LOG(WARNING) << "'printField()' calls are only allowed within the "
                    "'printFields()' method.";
    return;
  }
  std::stringstream ss;
  ss << "[";
  size_t offset = 0;
  for (const T& value : field) {
    ss << value << ", ";
    offset = 2;
  }
  std::string s = ss.str();
  s = s.substr(0, s.length()-offset).append("]");
  printFieldInternal(name, s);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::printField(const std::string& name,
                                 const std::map<std::string, T>& field) const {
  if (!meta_data_->messages) {
    LOG(WARNING) << "'printField()' calls are only allowed within the "
                    "'printFields()' method.";
    return;
  }
  std::stringstream ss;
  ss << "{";
  size_t offset = 0;
  for (auto it = field.begin(); it != field.end(); ++it) {
    ss << it->first << ": " << it->second << ", ";
    offset = 2;
  }
  std::string s = ss.str();
  s = s.substr(0, s.length()-offset).append("}");
  printFieldInternal(name, s);
}

template<typename ConfigT>
void Config<ConfigT>::printText(const std::string& text) const {
  if (!meta_data_->messages) {
    LOG(WARNING) << "'printText()' calls are only allowed within the "
                    "'printFields()' method.";
    return;
  }
  meta_data_->messages->emplace_back(text);
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::printFieldInternal(const std::string& name,
                                         const T& field) const {

  // The data.
  std::stringstream ss;
  ss << field;
  std::string f = ss.str();

  // The header is the field name.
  std::string header = std::string(meta_data_->indent, ' ') + name + ": ";
  while (header.length() > meta_data_->print_width) {
    // Linebreaks for too long lines.
    meta_data_->messages->emplace_back(header.substr(0,
                                                     meta_data_->print_width));
    header = header.substr(meta_data_->print_width);
  }
  if (header.length() < meta_data_->print_indent) {
    header.append(std::string(meta_data_->print_indent - header.length(), ' '));
  } else if (meta_data_->print_width - header.length() < f.length()) {
    meta_data_->messages->emplace_back(header);
    header = std::string(meta_data_->print_indent, ' ');
  }

  // First line could be shorter.
  int length = meta_data_->print_width - header.length();
  if (f.length() > length) {
    meta_data_->messages->emplace_back(header + f.substr(0, length));
    f = f.substr(length);

    // Fill the rest.
    length = meta_data_->print_width - meta_data_->print_indent;
    while (f.length() > length) {
      meta_data_->messages->emplace_back(
          std::string(meta_data_->print_indent, ' ')
              + f.substr(0, length));
      f = f.substr(length);
    }
    meta_data_->messages->emplace_back(
        std::string(meta_data_->print_indent, ' ')
            + f.substr(0, length));
  } else {
    meta_data_->messages->emplace_back(header.append(f));
  }
}

template<typename ConfigT>
void Config<ConfigT>::printConfigInternal(const std::string& name,
                                          const internal::ConfigBase* field) const {
  meta_data_->messages->emplace_back(
      std::string(meta_data_->indent, ' ') + name + ":");
  meta_data_->messages->emplace_back(field->toStringInternal(
      meta_data_->indent + Settings().default_subconfig_indent,
      meta_data_->print_width,
      meta_data_->print_indent));
}

template<typename ConfigT>
std::string Config<ConfigT>::toStringInternal(int indent,
                                              int print_width,
                                              int print_indent) const {
  int print_width_prev = meta_data_->print_width;
  int print_indent_prev = meta_data_->print_indent;
  int indent_prev = meta_data_->indent;
  meta_data_->print_width = print_width;
  meta_data_->print_indent = print_indent;
  meta_data_->indent = indent;

  meta_data_->messages.reset(new std::vector<std::string>());
  printFields();
  std::string result;
  for (const std::string& msg : *(meta_data_->messages)) {
    result.append("\n" + msg);
  }
  result = result.substr(1);
  meta_data_->messages.reset(nullptr);
  meta_data_->print_width = print_width_prev;
  meta_data_->print_indent = print_indent_prev;
  meta_data_->indent = indent_prev;
  return result;
};

template<typename ConfigT>
void Config<ConfigT>::setupFromParmaMap(const internal::ParamMap& params) {
  meta_data_->params = &params;
  fromRosParam();
  meta_data_->params = nullptr;
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::rosParamInternal(const std::string& name, T* param) {
  // Check scope and param map are valid.
  if (!meta_data_->params) {
    LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                    "'fromRosParam()' method, no param will be loaded.";
    return;
  }
  // Check the param is set.
  auto it = meta_data_->params->find(name);
  if(it == meta_data_->params->end()) {
    return;
  }
  // Set the param.
  if(!internal::xmlCast(it->second, param)){
    LOG(WARNING) << name_ << ": param '"
    << name << "' is set but could not be read.";
  }
}

template<typename ConfigT>
template<typename T>
void Config<ConfigT>::rosParamMapInternal(const std::string& name,
                                          std::map<std::string,T>* param) {
  // Check scope and param map are valid.
  if (!meta_data_->params) {
    LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                    "'fromRosParam()' method, no param will be loaded.";
    return;
  }
  // Check the param is set.
  std::map<std::string, T> values;
  for (const auto& v : *(meta_data_->params)) {
    if (v.first.find(name + "/") != 0) {
      continue;
    }
    std::string key = v.first.substr(name.length()+1);
    if (key.find('/') == std::string::npos) {
      T value;
      if (!internal::xmlCast(v.second, &value)) {
        LOG(WARNING) << name_ << ": param '"
                     << name << "' is set but could not be read.";
        return;
      }
      values[key] = value;
    }
  }
  *param = values;
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, int* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, float* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, double* param) {
  this->rosParamInternal(name, param);
}
template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, bool* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, std::string* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name, std::vector<int>* param) {
  this->rosParamInternal(name, param);
}
template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::vector<double>* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::vector<float>* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::vector<bool>* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::vector<std::string>* param) {
  this->rosParamInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::map<std::string, int>* param) {
  this->rosParamMapInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::map<std::string, double>* param) {
  this->rosParamMapInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::map<std::string, float>* param) {
  this->rosParamMapInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               std::map<std::string, bool>* param) {
  this->rosParamMapInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
              std::map<std::string, std::string>* param) {
  this->rosParamMapInternal(name, param);
}

template<typename ConfigT>
void Config<ConfigT>::rosParam(const std::string& name,
                               XmlRpc::XmlRpcValue* param) {
  // Check scope and param map are valid.
  if (!meta_data_->params) {
    LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                    "'fromRosParam()' method, no param will be loaded.";
    return;
  }
  // Check the param is set.
  auto it = meta_data_->params->find(name);
  if(it == meta_data_->params->end()) {
    return;
  }
  *param = it->second;
}

template<typename ConfigT>
template<typename ConfigType>
void Config<ConfigT>::rosParam(Config<ConfigType>* param, const std::string& sub_namespace) {

}
#ifdef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
template<typename ConfigT>
template<typename Scalar>
void Config<ConfigT>::rosParam(const std::string& name,
              kindr::minimal::QuatTransformationTemplate<Scalar>* param) {

}
#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED



}  // namespace config_utilities
#endif  // CONFIG_UTILITIES_CORE_HPP_



/**
 * ==================== ROS Tools ====================
 */
#ifdef CONFIG_UTILITIES_ROS_ENABLED
#ifndef CONFIG_UTILITIES_ROS_HPP_
#define CONFIG_UTILITIES_ROS_HPP_
namespace config_utilities {

// Tool to create configs from ROS
template<typename ConfigT>
ConfigT getConfigFromRos(const ros::NodeHandle& nh) {
  ConfigT config;
  if (!internal::isConfig(&config)) {
    LOG(ERROR) << "Can not 'getConfigFromRos()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  } else if (!dynamic_cast<Config<ConfigT>*>(&config)) {
    LOG(ERROR) << "Can not 'getConfigFromRos()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  }

  // Get params.
  internal::ParamMap params;
  std::vector<std::string> keys;
  XmlRpc::XmlRpcValue value;
  const std::string& ns = nh.getNamespace();
  nh.getParamNames(keys);
  for (std::string& key : keys) {
    if (key.find(ns) != 0) {
      continue;
    }
    key = key.substr(ns.length() + 1);
    nh.getParam(key, value);
    params[key] = value;
  }

  // Setup.
  internal::setupConfigFromParamMap(params, &config);
  return config;
}
}  // namespace config_utilities
#endif  // CONFIG_UTILITIES_ROS_HPP_
#endif  // CONFIG_UTILITIES_ROS_ENABLED


