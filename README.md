# Contact Leg Estimator

This repository implements a reduced 12-DOF dynamics model and a generalized-momentum disturbance observer for a wheel-legged quadruped platform. The model is derived symbolically in a Jupyter notebook, serialized to `saved_equations`, generated into C++, and used by the offline observer executable.

The current runtime model uses a frozen-inertia approximation: each leg inertia `I_c` is updated at every sample from the leg configuration, but the explicit inertia-rate force `dI_c/dt * q_dot` is not used in the C++ observer.

## Model coordinates

The generalized coordinate vector is

```text
q = [
  x, z, phi, psi,
  beta_lf, Rm_lf,
  beta_rf, Rm_rf,
  beta_rh, Rm_rh,
  beta_lh, Rm_lh
]
```

where:

- `x`, `z` are the floating-base position coordinates in the sagittal plane.
- `phi` is body roll.
- `psi` is body pitch.
- `beta_i` is the leg angular coordinate for leg `i`.
- `Rm_i` is the reduced prismatic leg length for leg `i`.
- Leg order is `lf`, `rf`, `rh`, `lh`.

The C++ offline processor maps logged data into this convention as:

```text
LF: beta_a,  Rm(theta_a)
RF: -beta_b, Rm(theta_b)
RH: -beta_c, Rm(theta_c)
LH: beta_d,  Rm(theta_d)
```

The RF and RH beta signs are negated so that all four legs use the same generalized-coordinate convention.

## Geometry

The floating-base origin is the body center of mass. The body position is

```text
p_B = [x, 0, z]^T
```

The body rotation used in the symbolic model is

```text
R_B^W = R_x(phi) R_y(psi)
```

The hip attachment locations in the body frame are

```text
LF: [ X_offset,  Y_offset, 0]^T
RF: [ X_offset, -Y_offset, 0]^T
RH: [-X_offset, -Y_offset, 0]^T
LH: [-X_offset,  Y_offset, 0]^T
```

The reduced leg center-of-mass location relative to the hip is

```text
r_m^B = [-Rm_i sin(beta_i), 0, -Rm_i cos(beta_i)]^T
```

Therefore the absolute leg center-of-mass position is

```text
p_m^W = [x, 0, z]^T + R_B^W (r_H^B + r_m^B)
```

and the absolute leg center-of-mass velocity is computed by differentiating `p_m^W` with respect to time.

## Kinetic and potential energy

The base kinetic energy is modeled as

```text
T_base =
  1/2 m_b (x_dot^2 + z_dot^2)
  + 1/2 I_Bxx phi_dot^2
  + 1/2 I_Byy psi_dot^2
```

For each leg, translational kinetic energy is computed from the absolute center-of-mass velocity:

```text
T_leg_trans = 1/2 m_l v_m^T v_m
```

The leg rotational kinetic energy uses the fact that `beta_i` and body pitch rotate about the same axis:

```text
T_leg_rot = 1/2 I_c_i (psi_dot + beta_dot_i)^2
```

This term is important. It adds the pitch-beta coupling term

```text
I_c_i psi_dot beta_dot_i
```

and produces the mass-matrix entries

```text
M(psi, beta_lf) += I_c_lf
M(psi, beta_rf) += I_c_rf
M(psi, beta_rh) += I_c_rh
M(psi, beta_lh) += I_c_lh
```

Potential energy is computed from the base and leg center-of-mass heights.

## Symbolic dynamics derivation

The symbolic derivation is in:

```text
python/motion_equation.ipynb
```

The Lagrangian is

```text
L = T - V
```

For every generalized coordinate `q_i`, the notebook applies the Lagrange-Euler equation:

```text
d/dt(∂L/∂q_dot_i) - ∂L/∂q_i
```

The equation of motion is organized as

```text
M(q, I_c) q_ddot + C(q, q_dot, I_c) q_dot + G(q) + D q_dot = S^T tau + tau_ext
```

In the exported runtime model:

```text
D = 0
```

The notebook retains a `D_exact` object for analysis of explicit inertia-rate effects, but the generated C++ intentionally exports a zero `D` matrix.

## Coriolis matrix convention

The notebook computes `C` using Christoffel symbols from the mass matrix. The generated `C` is a full matrix such that

```text
C_vec = C q_dot
```

The disturbance observer uses the generalized-momentum form and therefore evaluates

```text
C^T q_dot
```

This is intentional and should not be replaced by `C q_dot` in the observer.

## Actuation and selection matrix

Only the eight leg coordinates are actuated:

```text
tau = [
  tau_beta_lf, F_Rm_lf,
  tau_beta_rf, F_Rm_rf,
  tau_beta_rh, F_Rm_rh,
  tau_beta_lh, F_Rm_lh
]
```

The selection matrix has shape

```text
S^T: 12 x 8
```

and maps the eight actuator-space generalized forces into the full 12-DOF generalized force vector:

```text
S^T tau = [
  0, 0, 0, 0,
  tau_beta_lf, F_Rm_lf,
  tau_beta_rf, F_Rm_rf,
  tau_beta_rh, F_Rm_rh,
  tau_beta_lh, F_Rm_lh
]
```

The C++ observer constructs this matrix directly in `DisturbanceObserver::initialize_selection_matrix()`.

## Motor torque to reduced-coordinate force

The runtime state uses `Rm`, not the raw motor-side `theta`. The configuration relation is fitted as a polynomial:

```text
Rm = Rm(theta)
I_c = I_c(theta)
```

The coefficients are defined in:

```text
cpp/Config.hpp
```

At runtime:

- `theta_to_Rm(theta)` evaluates the fitted leg length.
- `theta_dot_to_Rm_dot(theta, theta_dot)` evaluates `dRm/dtheta * theta_dot`.
- `theta_to_Ic(theta)` evaluates the fitted leg inertia.

Motor torques are converted to reduced generalized forces by virtual work. With motor torque vector ordered as

```text
[tau_R, tau_L]^T
```

the implemented conversion is

```text
tau_beta = tau_L + tau_R
F_Rm = (tau_L - tau_R) / (dRm/dtheta)
```

If `|dRm/dtheta| < 1e-9`, the current implementation treats the configuration as singular and returns zero reduced force and torque for that leg.

## Code generation chain

The dynamics generation pipeline is:

```text
python/motion_equation.ipynb
  -> saved_equations/equations_dill.pkl
  -> python/generate_cpp_dynamics.py
  -> cpp/quadruped_dynamics.hpp
  -> cpp/quadruped_dynamics.cpp
```

The generated C++ API is:

```cpp
void compute_dynamics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& q_dot,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    Eigen::Ref<Eigen::MatrixXd> M,
    Eigen::Ref<Eigen::MatrixXd> C,
    Eigen::Ref<Eigen::VectorXd> G,
    Eigen::Ref<Eigen::MatrixXd> D
);
```

Inputs:

- `q`: 12-dimensional generalized coordinate vector.
- `q_dot`: 12-dimensional generalized velocity vector.
- `I_c`: four leg inertias ordered `[lf, rf, rh, lh]`.

Outputs:

- `M`: mass matrix.
- `C`: Coriolis matrix.
- `G`: gravity vector.
- `D`: inertia-rate matrix. Runtime output is currently zero.

## Runtime observer computation

The offline executable reads `data/cpp_test.csv`, converts the logged states and torques into the model coordinates, computes dynamics, and runs the disturbance observer.

The core observer equations are implemented in:

```text
cpp/DisturbanceObserver.cpp
```

At each sample:

```text
p_k = M(q_k) q_dot_k
S_T_tau = S^T tau
C_q_dot = C^T q_dot
Y_k = beta p_k + S_T_tau + C^T q_dot - G
Y_filtered_k = (1 - gamma) Y_k + gamma Y_filtered_{k-1}
tau_ext_hat = beta p_k - Y_filtered_k
```

The filter constants are

```text
gamma = exp(-lambda dt)
beta = (1 - gamma) / (gamma dt)
```

where `lambda` is derived from the observer cutoff frequency.

## Build and run

Configure and build with CMake:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=C:\tmp\eigen-3.3.9\install
cmake --build build --config Release
```

Run the offline observer:

```powershell
.\build\quadruped_observer.exe
```

The executable reads:

```text
data/cpp_test.csv
```

and writes observer logs under:

```text
output_data/
```

`output_data/` is ignored by Git because these files are generated analysis outputs.

## Current modeling assumptions

- The model keeps `x`, `z`, roll, and pitch, but omits lateral translation and yaw.
- IMU angular velocities are used directly as roll and pitch rates in the runtime model.
- Leg `beta_i` and body pitch are treated as same-axis rotations.
- `I_c(theta)` is updated every sample, but explicit `dI_c/dt` generalized forces are omitted in runtime.
- `D` is generated as zero and is not used by the disturbance observer.
- The fitted `Rm(theta)` Jacobian must stay away from singular regions for reliable force conversion.

## Important files

```text
python/motion_equation.ipynb       Symbolic dynamics derivation
saved_equations/equations_dill.pkl Serialized symbolic dynamics
python/generate_cpp_dynamics.py    C++ dynamics generator
cpp/quadruped_dynamics.cpp         Generated dynamics implementation
cpp/quadruped_dynamics.hpp         Generated dynamics API
cpp/DisturbanceObserver.cpp        Generalized-momentum observer
cpp/main.cpp                       Offline CSV processing and runtime mapping
cpp/Config.hpp                     Physical constants and fitted coefficients
```
