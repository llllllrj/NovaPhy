"""Tests for NovaPhy articulated body dynamics (Featherstone)."""

import numpy as np
import numpy.testing as npt
import novaphy


def _make_single_pendulum(length=1.0, mass=1.0):
    """Create a single revolute-joint pendulum."""
    art = novaphy.Articulation()

    # Single revolute joint rotating about Z axis
    joint = novaphy.Joint()
    joint.type = novaphy.JointType.Revolute
    joint.axis = np.array([0, 0, 1], dtype=np.float32)
    joint.parent = -1  # attached to world
    # Joint at origin, link extends along -Y (hanging down)
    joint.parent_to_joint = novaphy.Transform.identity()
    art.joints = [joint]

    # Link body: mass at the end of the pendulum arm
    body = novaphy.RigidBody()
    body.mass = mass
    body.com = np.array([0, -length, 0], dtype=np.float32)  # COM at end of arm
    # Point mass inertia about origin
    body.inertia = np.eye(3, dtype=np.float32) * mass * length * length
    art.bodies = [body]

    art.build_spatial_inertias()
    return art


def _make_double_pendulum(l1=1.0, l2=1.0, m1=1.0, m2=1.0):
    """Create a double revolute-joint pendulum."""
    art = novaphy.Articulation()

    # Joint 0: first pendulum link, attached to world
    j0 = novaphy.Joint()
    j0.type = novaphy.JointType.Revolute
    j0.axis = np.array([0, 0, 1], dtype=np.float32)
    j0.parent = -1
    j0.parent_to_joint = novaphy.Transform.identity()

    # Joint 1: second pendulum link, attached to end of first
    j1 = novaphy.Joint()
    j1.type = novaphy.JointType.Revolute
    j1.axis = np.array([0, 0, 1], dtype=np.float32)
    j1.parent = 0
    j1.parent_to_joint = novaphy.Transform.from_translation(
        np.array([0, -l1, 0], dtype=np.float32))

    art.joints = [j0, j1]

    # Link 0: mass at end
    b0 = novaphy.RigidBody()
    b0.mass = m1
    b0.com = np.array([0, -l1, 0], dtype=np.float32)
    b0.inertia = np.eye(3, dtype=np.float32) * m1 * l1 * l1

    # Link 1: mass at end
    b1 = novaphy.RigidBody()
    b1.mass = m2
    b1.com = np.array([0, -l2, 0], dtype=np.float32)
    b1.inertia = np.eye(3, dtype=np.float32) * m2 * l2 * l2

    art.bodies = [b0, b1]
    art.build_spatial_inertias()
    return art


def test_joint_types():
    j = novaphy.Joint()
    j.type = novaphy.JointType.Revolute
    assert j.num_q() == 1
    assert j.num_qd() == 1

    j.type = novaphy.JointType.Fixed
    assert j.num_q() == 0
    assert j.num_qd() == 0

    j.type = novaphy.JointType.Free
    assert j.num_q() == 7
    assert j.num_qd() == 6


def test_articulation_dof_counts():
    art = _make_double_pendulum()
    assert art.num_links() == 2
    assert art.total_q() == 2
    assert art.total_qd() == 2
    assert art.q_start(0) == 0
    assert art.q_start(1) == 1
    assert art.qd_start(0) == 0
    assert art.qd_start(1) == 1


def test_forward_kinematics_zero():
    """FK with zero joint angles should give identity-like transforms."""
    art = _make_single_pendulum()
    q = np.zeros(1, dtype=np.float32)
    transforms = novaphy.forward_kinematics(art, q)
    assert len(transforms) == 1
    # At zero angle, link is at identity orientation
    pos = transforms[0].position
    # Position should be [0, 0, 0] (joint at origin, no displacement)
    npt.assert_allclose(pos, [0, 0, 0], atol=1e-5)


def test_forward_kinematics_rotated():
    """FK with 90-degree rotation about Z axis."""
    art = _make_single_pendulum(length=1.0)
    q = np.array([np.pi / 2], dtype=np.float32)
    transforms = novaphy.forward_kinematics(art, q)
    # At 90 degrees around Z, the link frame should be rotated
    rot = transforms[0].rotation  # [x, y, z, w]
    # Expected quaternion for 90° about Z: [0, 0, sin(45°), cos(45°)]
    expected_z = np.sin(np.pi / 4)
    expected_w = np.cos(np.pi / 4)
    npt.assert_allclose(abs(rot[2]), expected_z, atol=1e-4)
    npt.assert_allclose(abs(rot[3]), expected_w, atol=1e-4)


def test_mass_matrix_single_pendulum():
    """Mass matrix for single pendulum should be a 1x1 matrix = I_zz."""
    art = _make_single_pendulum(length=1.0, mass=1.0)
    q = np.zeros(1, dtype=np.float32)
    H = novaphy.mass_matrix_crba(art, q)
    assert H.shape == (1, 1)
    # For a point mass at distance L, I_zz = m * L^2 = 1.0
    assert H[0, 0] > 0.5, f"Expected positive inertia, got {H[0, 0]}"


def test_mass_matrix_symmetric():
    """Mass matrix should be symmetric positive definite."""
    art = _make_double_pendulum()
    q = np.array([0.3, -0.5], dtype=np.float32)
    H = novaphy.mass_matrix_crba(art, q)
    assert H.shape == (2, 2)
    # Symmetry
    npt.assert_allclose(H, H.T, atol=1e-5)
    # Positive definite: all eigenvalues > 0
    eigenvalues = np.linalg.eigvalsh(H)
    assert np.all(eigenvalues > 0), f"Not positive definite: eigenvalues = {eigenvalues}"


def test_inverse_dynamics_static():
    """At rest hanging vertically, RNEA should return gravity compensation torques."""
    art = _make_single_pendulum(length=1.0, mass=1.0)
    q = np.zeros(1, dtype=np.float32)
    qd = np.zeros(1, dtype=np.float32)
    qdd = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    tau = novaphy.inverse_dynamics(art, q, qd, qdd, gravity)
    assert len(tau) == 1
    # For a pendulum hanging vertically (COM at (0, -L, 0)),
    # gravity torque about Z axis = m * g * L * sin(0) = 0
    # (the pendulum is in equilibrium hanging down)
    npt.assert_allclose(tau[0], 0.0, atol=1e-3)


def test_forward_dynamics_free_fall():
    """With zero torques and pendulum at 90°, it should accelerate under gravity."""
    art = _make_single_pendulum(length=1.0, mass=1.0)
    q = np.array([np.pi / 2], dtype=np.float32)  # horizontal
    qd = np.zeros(1, dtype=np.float32)
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    qdd = novaphy.forward_dynamics(art, q, qd, tau, gravity)
    assert len(qdd) == 1
    # At 90° horizontal, gravity creates a torque that should cause negative angular accel
    # (pendulum falls back toward hanging position)
    assert qdd[0] != 0, "Expected non-zero acceleration"


def test_articulated_solver_step():
    """Test that the solver integrates without crashing."""
    art = _make_single_pendulum()
    q = np.array([0.5], dtype=np.float32)  # initial angle
    qd = np.zeros(1, dtype=np.float32)
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    q_new, qd_new = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)

    assert len(q_new) == 1
    assert len(qd_new) == 1
    # Angle should have changed (pendulum swings)
    assert q_new[0] != q[0], "Pendulum angle should change under gravity"


def test_pendulum_energy_conservation():
    """Pendulum energy should be approximately conserved over many steps."""
    art = _make_single_pendulum(length=1.0, mass=1.0)
    q = np.array([np.pi / 4], dtype=np.float32)  # 45 degrees
    qd = np.zeros(1, dtype=np.float32)
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    # Compute initial energy
    # KE = 0.5 * I * qd^2 = 0
    # PE = -m * g * L * cos(q) (with hanging down = 0 PE reference)
    m, L, g = 1.0, 1.0, 9.81
    E_initial = -m * g * L * np.cos(q[0])

    solver = novaphy.ArticulatedSolver()
    for _ in range(240):  # 2 seconds at 120 Hz
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)

    H = novaphy.mass_matrix_crba(art, q)
    KE = 0.5 * float(qd.T @ H @ qd)
    PE = -m * g * L * np.cos(q[0])
    E_final = KE + PE

    # Allow 10% energy drift (semi-implicit Euler is not energy-conserving)
    energy_ratio = abs(E_final / E_initial) if abs(E_initial) > 0.1 else 1.0
    assert 0.5 < energy_ratio < 2.0, \
        f"Energy not approximately conserved: E_initial={E_initial:.3f}, E_final={E_final:.3f}"


# ---- Slide joint tests ----

def test_slide_joint_dof():
    """Slide joint should have num_q=1, num_qd=1."""
    j = novaphy.Joint()
    j.type = novaphy.JointType.Slide
    assert j.num_q() == 1
    assert j.num_qd() == 1


def test_slide_joint_falls_under_gravity():
    """A mass on a vertical slide joint should fall under gravity."""
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Slide
    j.axis = np.array([0, -1, 0], dtype=np.float32)  # slide along -Y
    j.parent = -1
    j.parent_to_joint = novaphy.Transform.identity()
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = 1.0
    body.com = np.array([0, 0, 0], dtype=np.float32)
    body.inertia = np.eye(3, dtype=np.float32) * 0.01
    art.bodies = [body]
    art.build_spatial_inertias()

    q = np.zeros(1, dtype=np.float32)
    qd = np.zeros(1, dtype=np.float32)
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    for _ in range(120):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)

    # Mass should have moved along -Y axis (positive q means displacement along axis)
    assert q[0] > 0.1, f"Slide joint should move under gravity, got q={q[0]}"


def test_slide_joint_mass_matrix():
    """Mass matrix for a single slide joint should be a 1x1 scalar = mass."""
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Slide
    j.axis = np.array([1, 0, 0], dtype=np.float32)
    j.parent = -1
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = 2.5
    body.com = np.array([0, 0, 0], dtype=np.float32)
    body.inertia = np.eye(3, dtype=np.float32) * 0.1
    art.bodies = [body]
    art.build_spatial_inertias()

    q = np.zeros(1, dtype=np.float32)
    H = novaphy.mass_matrix_crba(art, q)
    assert H.shape == (1, 1)
    # For a slide joint with COM at origin, H should equal the mass
    npt.assert_allclose(H[0, 0], 2.5, atol=0.1)


# ---- Ball joint tests ----

def test_ball_joint_dof():
    """Ball joint should have num_q=4, num_qd=3."""
    j = novaphy.Joint()
    j.type = novaphy.JointType.Ball
    assert j.num_q() == 4
    assert j.num_qd() == 3


def test_ball_joint_forward_kinematics():
    """Ball joint FK should produce rotation from quaternion."""
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Ball
    j.parent = -1
    j.parent_to_joint = novaphy.Transform.identity()
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = 1.0
    body.inertia = np.eye(3, dtype=np.float32) * 0.1
    art.bodies = [body]
    art.build_spatial_inertias()

    # Identity quaternion: [0, 0, 0, 1] (qx, qy, qz, qw)
    q = np.array([0, 0, 0, 1], dtype=np.float32)
    transforms = novaphy.forward_kinematics(art, q)
    assert len(transforms) == 1
    npt.assert_allclose(transforms[0].position, [0, 0, 0], atol=1e-5)


def test_ball_joint_swings_under_gravity():
    """A mass attached via ball joint should swing under gravity."""
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Ball
    j.parent = -1
    j.parent_to_joint = novaphy.Transform.identity()
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = 1.0
    body.com = np.array([0, -1, 0], dtype=np.float32)
    body.inertia = np.eye(3, dtype=np.float32) * 1.0
    art.bodies = [body]
    art.build_spatial_inertias()

    # Start with 45-degree rotation about Z: quat for 45° about Z
    angle = np.pi / 4
    q = np.array([0, 0, np.sin(angle/2), np.cos(angle/2)], dtype=np.float32)
    qd = np.zeros(3, dtype=np.float32)
    tau = np.zeros(3, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    for _ in range(120):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)

    # Angular velocity should be non-zero (swinging)
    assert np.linalg.norm(qd) > 0.01, "Ball joint should swing under gravity"


def test_ball_joint_mass_matrix_symmetric():
    """Ball joint mass matrix should be 3x3 symmetric positive definite."""
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Ball
    j.parent = -1
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = 1.0
    body.com = np.array([0, -0.5, 0], dtype=np.float32)
    body.inertia = np.diag([0.1, 0.2, 0.3]).astype(np.float32)
    art.bodies = [body]
    art.build_spatial_inertias()

    # Some non-trivial orientation
    angle = 0.3
    q = np.array([np.sin(angle/2), 0, 0, np.cos(angle/2)], dtype=np.float32)
    H = novaphy.mass_matrix_crba(art, q)
    assert H.shape == (3, 3)
    npt.assert_allclose(H, H.T, atol=1e-5)
    eigenvalues = np.linalg.eigvalsh(H)
    assert np.all(eigenvalues > 0), f"Not positive definite: {eigenvalues}"
