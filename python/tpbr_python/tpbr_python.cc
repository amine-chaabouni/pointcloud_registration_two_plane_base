/**
 * Authors: Amine Chaabouni
 * See LICENSE for the license information
 */

#include <string>
#include <sstream>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "../../tools/include/solver.h"


namespace py = pybind11;

/**
 * Python interface with pybind11
 */
PYBIND11_MODULE(tpbr_python, m) {
    m.doc() = "Python binding for Two Plane Based Registration";

    py::class_<PointT> PointT(m, "PointT");

    PointT.def(py::init<>())
            .def_readwrite("x", &PointT::x)
            .def_readwrite("y", &PointT::y)
            .def_readwrite("z", &PointT::z);

    // Python bound for teaser::RobustRegistraionSolver
    py::class_<Solver> Solver(m, "TwoPlaneBasedRegistrationSolver");

    // Python bound for teaser::RobustRegistrationSolver functions
    Solver.def(py::init<>())
            .def(py::init<const Solver::Params &>())
            .def("solve", &Solver::solve)
            .def("get_source_nb_planes", &Solver::getSourceNbPlanes)
            .def("get_target_nb_planes", &Solver::getTargetNbPlanes)
            .def("get_source_nb_bases", &Solver::getSourceNbBases)
            .def("get_target_nb_bases", &Solver::getTargetNbBases)
            .def("get_source_cloud", &Solver::getSourceCloud)
            .def("get_target_cloud", &Solver::getTargetCloud);

    // Python bound for Solver::Params
    py::class_<Solver::Params>(Solver, "Params")
            .def(py::init<>())
            .def_readwrite("source_cloud_path", &Solver::Params::source_cloud_path)
            .def_readwrite("target_cloud_path", &Solver::Params::target_cloud_path)
            .def_readwrite("source_resolution", &Solver::Params::source_resolution)
            .def_readwrite("source_min_point_per_voxel", &Solver::Params::source_min_point_per_voxel)
            .def_readwrite("target_resolution", &Solver::Params::target_resolution)
            .def_readwrite("target_min_point_per_voxel", &Solver::Params::target_min_point_per_voxel)
            .def_readwrite("planarity_score", &Solver::Params::planarity_score)
            .def_readwrite("min_angle", &Solver::Params::min_angle)
            .def_readwrite("max_angle", &Solver::Params::max_angle)

            .def("__repr__", [](const Solver::Params &a) {
                std::ostringstream print_string;

                print_string << "<Params with source_cloud_path=" << a.source_cloud_path << "\n"
                             << "target_cloud_path=" << a.target_cloud_path << "\n"
                             << "source_resolution=" << a.source_resolution << "\n"
                             << "source_min_point_per_voxel=" << a.source_min_point_per_voxel << "\n"
                             << "target_resolution=" << a.target_resolution << "\n"
                             << "target_min_point_per_voxel=" << a.target_min_point_per_voxel << "\n"
                             << "planarity_score=" << a.planarity_score << "\n"
                             << "min_angle=" << a.min_angle << "\n"
                             << "max_angle=" << a.max_angle << "\n"
                             << ">";
                return print_string.str();
            });


}