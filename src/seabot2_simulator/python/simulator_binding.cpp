#include <pybind11/pybind11.h>
#include "seabot2_simulator/simulator.h"
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(seabot2py, m) {
    m.doc() = "Python binding of seabot2_simulator";

    py::class_<Simulator>(m, "Simulator", py::dynamic_attr())
            .def(py::init<>())
            .def("run_simulation", &Simulator::run_simulation)
            .def_readwrite("memory_time", &Simulator::memory_time)
            .def_readwrite("memory_piston_position", &Simulator::memory_piston_position)
            .def_readwrite("memory_piston_velocity", &Simulator::memory_piston_velocity)
            .def_readwrite("memory_velocity", &Simulator::memory_velocity)
            .def_readwrite("memory_depth", &Simulator::memory_depth)
            ;

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}