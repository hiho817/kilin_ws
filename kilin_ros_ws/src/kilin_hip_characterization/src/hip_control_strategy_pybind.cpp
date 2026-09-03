#include "kilin_hip_characterization/hip_control_strategy.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using kilin_hip_characterization::HipControlInput;
using kilin_hip_characterization::HipControlOutput;
using kilin_hip_characterization::HipControlStrategy;
using kilin_hip_characterization::HipControlStrategyConfig;

PYBIND11_MODULE(_hip_control_strategy, module)
{
  module.doc() = "Clock-free Kilin hip fine-tuning strategy core.";

  py::class_<HipControlStrategyConfig>(module, "StrategyConfig")
    .def(py::init<>())
    .def_readwrite("enabled", &HipControlStrategyConfig::enabled)
    .def_readwrite("pid_schedule_enabled", &HipControlStrategyConfig::pid_schedule_enabled)
    .def_readwrite("support_end", &HipControlStrategyConfig::support_end)
    .def_readwrite("lift_start", &HipControlStrategyConfig::lift_start)
    .def_readwrite("support_kp", &HipControlStrategyConfig::support_kp)
    .def_readwrite("support_ki", &HipControlStrategyConfig::support_ki)
    .def_readwrite("support_kd", &HipControlStrategyConfig::support_kd)
    .def_readwrite("lift_kp", &HipControlStrategyConfig::lift_kp)
    .def_readwrite("lift_ki", &HipControlStrategyConfig::lift_ki)
    .def_readwrite("lift_kd", &HipControlStrategyConfig::lift_kd)
    .def_readwrite("lift_start_inward_ff", &HipControlStrategyConfig::lift_start_inward_ff)
    .def_readwrite("lift_max_inward_ff", &HipControlStrategyConfig::lift_max_inward_ff)
    .def_readwrite("lift_ramp_up_nm_s", &HipControlStrategyConfig::lift_ramp_up_nm_s)
    .def_readwrite("lift_ramp_down_nm_s", &HipControlStrategyConfig::lift_ramp_down_nm_s)
    .def_readwrite("reset_lift_on_support", &HipControlStrategyConfig::reset_lift_on_support)
    .def_readwrite("support_inward_ff", &HipControlStrategyConfig::support_inward_ff)
    .def_readwrite("apply_rate_nm_s", &HipControlStrategyConfig::apply_rate_nm_s)
    .def_readwrite("release_rate_nm_s", &HipControlStrategyConfig::release_rate_nm_s)
    .def_readwrite("max_abs_ff", &HipControlStrategyConfig::max_abs_ff)
    .def_readwrite("kp_to_lift_rate_per_s", &HipControlStrategyConfig::kp_to_lift_rate_per_s)
    .def_readwrite("kp_to_support_rate_per_s", &HipControlStrategyConfig::kp_to_support_rate_per_s)
    .def_readwrite("ki_to_lift_rate_per_s", &HipControlStrategyConfig::ki_to_lift_rate_per_s)
    .def_readwrite("ki_to_support_rate_per_s", &HipControlStrategyConfig::ki_to_support_rate_per_s)
    .def_readwrite("kd_to_lift_rate_per_s", &HipControlStrategyConfig::kd_to_lift_rate_per_s)
    .def_readwrite("kd_to_support_rate_per_s", &HipControlStrategyConfig::kd_to_support_rate_per_s);

  py::class_<HipControlInput>(module, "ControlSample")
    .def(py::init<>())
    .def_readwrite("module", &HipControlInput::module)
    .def_readwrite("motor_position", &HipControlInput::motor_position)
    .def_readwrite("position_diff", &HipControlInput::position_diff)
    .def_readwrite("commanded_motor_position", &HipControlInput::commanded_motor_position)
    .def_readwrite("now_s", &HipControlInput::now_s)
    .def_readwrite("active", &HipControlInput::active)
    .def_readwrite("normal_phase", &HipControlInput::normal_phase)
    .def_readwrite("outward_range", &HipControlInput::outward_range)
    .def_readwrite("kp", &HipControlInput::kp)
    .def_readwrite("ki", &HipControlInput::ki)
    .def_readwrite("kd", &HipControlInput::kd);

  py::class_<HipControlOutput>(module, "ControlResult")
    .def_readonly("kp", &HipControlOutput::kp)
    .def_readonly("ki", &HipControlOutput::ki)
    .def_readonly("kd", &HipControlOutput::kd)
    .def_readonly("motor_ff", &HipControlOutput::motor_ff)
    .def_readonly("normalized_diff", &HipControlOutput::normalized_diff)
    .def_readonly("pid_blend", &HipControlOutput::pid_blend)
    .def_readonly("target_inward_ff", &HipControlOutput::target_inward_ff)
    .def_readonly("applied_inward_ff", &HipControlOutput::applied_inward_ff)
    .def_readonly("lift_dwell_s", &HipControlOutput::lift_dwell_s)
    .def_readonly("lift_latched", &HipControlOutput::lift_latched)
    .def_readonly("valid", &HipControlOutput::valid);

  py::class_<HipControlStrategy>(module, "FineTuneStrategy")
    .def(py::init<>())
    .def("configure", &HipControlStrategy::configure)
    .def("step", &HipControlStrategy::update,
      "Synchronously evaluate one caller-owned control sample.")
    .def("reset", &HipControlStrategy::reset)
    .def("reset_all", &HipControlStrategy::reset_all);
}
