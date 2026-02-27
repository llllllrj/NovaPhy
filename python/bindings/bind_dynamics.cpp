#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/core/articulation.h"
#include "novaphy/core/joint.h"
#include "novaphy/dynamics/articulated_solver.h"
#include "novaphy/dynamics/featherstone.h"

namespace py = pybind11;
using namespace novaphy;

void bind_dynamics(py::module_& m) {
    // --- JointType ---
    py::enum_<JointType>(m, "JointType")
        .value("Revolute", JointType::Revolute)
        .value("Fixed", JointType::Fixed)
        .value("Free", JointType::Free)
        .value("Slide", JointType::Slide)
        .value("Ball", JointType::Ball);

    // --- Joint ---
    py::class_<Joint>(m, "Joint")
        .def(py::init<>())
        .def_readwrite("type", &Joint::type)
        .def_readwrite("axis", &Joint::axis)
        .def_readwrite("parent", &Joint::parent)
        .def_readwrite("parent_to_joint", &Joint::parent_to_joint)
        .def("num_q", &Joint::num_q)
        .def("num_qd", &Joint::num_qd);

    // --- Articulation ---
    py::class_<Articulation>(m, "Articulation")
        .def(py::init<>())
        .def_readwrite("joints", &Articulation::joints)
        .def_readwrite("bodies", &Articulation::bodies)
        .def("num_links", &Articulation::num_links)
        .def("total_q", &Articulation::total_q)
        .def("total_qd", &Articulation::total_qd)
        .def("q_start", &Articulation::q_start, py::arg("link"))
        .def("qd_start", &Articulation::qd_start, py::arg("link"))
        .def("build_spatial_inertias", &Articulation::build_spatial_inertias);

    // --- Featherstone algorithms ---
    m.def("forward_kinematics", [](const Articulation& model, const VecXf& q) {
        std::vector<SpatialTransform> X_J, X_up;
        std::vector<Transform> X_world;
        featherstone::forward_kinematics(model, q, X_J, X_up, X_world);
        return X_world;
    }, py::arg("model"), py::arg("q"),
    "Forward kinematics: returns world transforms for each link.");

    m.def("inverse_dynamics", [](const Articulation& model,
                                 const VecXf& q, const VecXf& qd,
                                 const VecXf& qdd, const Vec3f& gravity) {
        return featherstone::inverse_dynamics(model, q, qd, qdd, gravity);
    }, py::arg("model"), py::arg("q"), py::arg("qd"),
       py::arg("qdd"), py::arg("gravity"),
    "Inverse dynamics (RNEA): returns joint torques.");

    m.def("mass_matrix_crba", [](const Articulation& model, const VecXf& q) {
        return featherstone::mass_matrix(model, q);
    }, py::arg("model"), py::arg("q"),
    "Mass matrix (CRBA): returns joint-space inertia matrix H(q).");

    m.def("forward_dynamics", [](const Articulation& model,
                                 const VecXf& q, const VecXf& qd,
                                 const VecXf& tau, const Vec3f& gravity) {
        return featherstone::forward_dynamics(model, q, qd, tau, gravity);
    }, py::arg("model"), py::arg("q"), py::arg("qd"),
       py::arg("tau"), py::arg("gravity"),
    "Forward dynamics: returns joint accelerations.");

    // --- ArticulatedSolver ---
    py::class_<ArticulatedSolver>(m, "ArticulatedSolver")
        .def(py::init<>())
        .def("step", [](ArticulatedSolver& self, const Articulation& model,
                        VecXf q, VecXf qd, const VecXf& tau,
                        const Vec3f& gravity, float dt) {
            self.step(model, q, qd, tau, gravity, dt);
            return std::make_pair(q, qd);
        }, py::arg("model"), py::arg("q"), py::arg("qd"),
           py::arg("tau"), py::arg("gravity"), py::arg("dt"),
        "Step articulated body forward. Returns (q, qd).");
}
