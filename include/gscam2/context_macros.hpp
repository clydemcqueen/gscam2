#ifndef CONTEXT_MACROS_HPP
#define CONTEXT_MACROS_HPP

#include <algorithm>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

// A set of macros that help define parameters in ROS2 nodes.
//
//  1) Define a master macro with containing the full list of parameters.
//  2) Define the member parameter variables with the CXT_MACRO_DEFINE_MEMBER macro.
//  3) Initialize the member parameter variables with the CXT_MACRO_LOAD_PARAMETER and CXT_MACRO_INIT_PARAMETERS macros.
//  4) Setup dynamic member variable updates with the CXT_MACRO_PARAMETER_CHANGED and
//      CXT_MACRO_REGISTER_PARAMETERS_CHANGED macros.
//  5) Display the values of all parameters with the CXT_MACRO_LOG_PARAMETER macro.
//

// From Peter Mullen, https://github.com/ptrmu/ros2_shared
// This file BSD 3-Clause License

// ==============================================================================
// Define parameters
// ==============================================================================

// Define CXT_MACRO_MEMBER to CXT_MACRO_DEFINE_MEMBER before declaring members
#define CXT_MACRO_DEFINE_MEMBER(n, t, d) t n##_{d};

#define CXT_MACRO_DEFINE_MEMBERS(all_params) \
  all_params \
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cxt_macro_callback_handle_{};

// ==============================================================================
// Declare parameters
// ==============================================================================

// Define CXT_MACRO_MEMBER to CXT_MACRO_LOAD_PARAMETER before invoking CXT_MACRO_INIT_PARAMETERS
#define CXT_MACRO_LOAD_PARAMETER(node_ref, cxt_ref, n, t, d) \
  cxt_ref.n##_ = node_ref.declare_parameter(#n, cxt_ref.n##_);

// Initialize the parameter members from the node
#define CXT_MACRO_INIT_PARAMETERS(all_params, validate_func) \
all_params \
validate_func(); \


// ==============================================================================
// Register for parameter changed notification (by external sources)
// ==============================================================================

// Define CXT_MACRO_MEMBER to CXT_MACRO_PARAMETER_CHANGED before invoking CXT_MACRO_REGISTER_PARAMETERS_CHANGED
// Notice that the_logger and param_set are expected to be defined and initialized
// in the CXT_MACRO_REGISTER_PARAMETERS_CHANGED macro
#define CXT_MACRO_PARAMETER_CHANGED(n, t) \
if (parameter.get_name() == #n) {\
  param_set = true; \
  _c.n##_ = parameter.get_value<t>(); \
  RCLCPP_INFO(the_logger, "Parameter %s value changed to %s", #n, \
  rclcpp::to_string(rclcpp::ParameterValue{_c.n##_}).c_str()); \
}

// Register for parameter changed notifications
#define CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_ref, cxt_ref, all_params, validate_func) \
cxt_ref.cxt_macro_callback_handle_ = node_ref.add_on_set_parameters_callback( \
[this, &_c = cxt_ref, the_logger = node_ref.get_logger()]\
(const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult\
{\
  auto result = rcl_interfaces::msg::SetParametersResult(); \
  bool param_set{false}; \
  for (const auto &parameter : parameters) { \
    all_params \
  } \
  if (param_set) { validate_func(); }\
  result.successful = true; \
  return result; \
});


// ==============================================================================
// Log the current parameter values - often done at startup
// ==============================================================================

// Define CXT_MACRO_MEMBER to CXT_MACRO_LOG_PARAMETER before logging the current value of the parameters
#define CXT_MACRO_LOG_PARAMETER(rcl_macro, logger, cxt_ref, n, t, d) \
  rcl_macro(logger, "%s = %s", #n, \
  rclcpp::to_string(rclcpp::ParameterValue{cxt_ref.n##_}).c_str());

#define CXT_MACRO_LOG_SORTED_PARAMETER(cxt_ref, n, t, d)  ps.emplace_back(std::string(#n).append(" = ")\
  .append(rclcpp::to_string(rclcpp::ParameterValue{cxt_ref.n##_}).c_str()));

#define CXT_MACRO_LOG_SORTED_PARAMETERS(rcl_macro, logger, title, all_params) \
{ \
  std::vector<std::string> ps{}; \
  all_params \
  std::sort(ps.begin(), ps.end()); \
  std::string s{title}; \
  for (auto &p : ps) { \
    s.append("\n").append(p); \
  } \
  rcl_macro(logger, s.c_str()); \
}


// ==============================================================================
// Check for parameters that were on the command line but are not defined. (Often a command line mis-type)
// ==============================================================================

// Define CXT_MACRO_MEMBER to CXT_MACRO_CHECK_CMDLINE_PARAMETER and then use the
// CXT_MACRO_CHECK_CMDLINE_PARAMETERS marco to display invalid/undefined command line parameters.
#define CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d) if (npo.first == #n) continue;

#define CXT_MACRO_CHECK_CMDLINE_PARAMETERS(node_ref, all_params) {\
  auto npi = get_node_parameters_interface(); \
  auto npos = npi->get_parameter_overrides(); \
  for (auto &npo : npos) { \
    all_params \
    if (npo.first == "use_sim_time") continue; \
    RCLCPP_INFO(node_ref.get_logger(), "**** ERROR: Undefined command line parameter: %s", npo.first.c_str()); \
  } \
}

// Use CXT_MACRO_SET_PARAMETER to set the local and node's parameter value
#define CXT_MACRO_SET_PARAMETER(node_ref, cxt_ref, n, d) \
  do { \
  cxt_ref.n##_ = d; \
  node_ref.set_parameter(rclcpp::Parameter(#n, d)); \
  } while (false)

#endif // CONTEXT_MACROS_HPP
