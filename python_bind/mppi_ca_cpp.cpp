#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pytensor.hpp>
#include "../include/controller.hpp"
#include "../include/orca_lines.hpp"
#include "converters.hpp"

namespace py = pybind11;

PYBIND11_MODULE(mppi_ca_cpp, m) {
	xt::import_numpy();
	py::class_<mppica::AgentParams>(m, "AgentParams")
			.def(py::init<>());
	py::class_<mppica::MPPIParams>(m, "AlgParams")
			.def(py::init<>());
	py::class_<mppica::Controller>(m, "Controller")
			.def(py::init<>())
			.def("next_step", &mppica::Controller::nextStep)
			.def("set_agent_params", &mppica::Controller::setAgentParams)
            .def("set_obstacles", &mppica::Controller::setObstacles, py::arg("polygons"))
			.def("set_anchor_path", &mppica::Controller::setAnchorPath, py::arg("points"),
				 "Set an ordered polygonal anchor path, or pass [] to disable path following.")
			.def("set_alg_params", &mppica::Controller::setAlgParams);
    m.def("polygon_collision", &polygonCollision, py::arg("start"), py::arg("end"), py::arg("polygon"), py::arg("radius") = 0.0);
    m.def("set_random_seed", [](unsigned int seed) { xt::random::seed(seed); },
          py::arg("seed"), "Seed the shared MPPI sampler after constructing controllers.");

	m.def("convert_agent_params", &convertAgentParams, "Coverts agent params");
	m.def("convert_alg_params", &convertAlgParams, "Coverts alg parameters");
	m.def("compute_orca_lines", &computeORCALinesXT, "Computes ORCA lines for agents");
}
