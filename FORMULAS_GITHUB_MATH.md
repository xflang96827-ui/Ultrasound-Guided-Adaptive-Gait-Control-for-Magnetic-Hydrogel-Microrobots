# FORMULAS — GitHub-Compatible Version

> This file uses GitHub's fenced math syntax:
>
> ```markdown
> ```math
> a^2+b^2=c^2
> ```
> ```
>
> This is more robust on GitHub than `\[` `\]` or some `$$ ... $$` blocks, especially when equations contain underscores, superscripts, matrices, or line breaks.

---

# 1. Ultrasound Localization

## 1.1 Inter-Frame Difference Map

```math
D_t(x,y)
=
\alpha \left| I_t(x,y)-I_{t-1}(x,y) \right|
+
\beta \left| I_t(x,y)-I_0(x,y) \right|
```

```math
\alpha+\beta=1
```

where:

- `I_t(x,y)` is the current ultrasound frame.
- `I_{t-1}(x,y)` is the previous frame.
- `I_0(x,y)` is the initial background frame.
- `alpha` emphasizes instantaneous motion.
- `beta` suppresses background noise.

---

# 2. RRT* Path Planning

## 2.1 State Space

```math
\chi
```

```math
\chi_{obs}
```

```math
\chi_{boundary}
```

```math
\chi_{start}, \quad \chi_{goal}
```

```math
\sigma
```

---

## 2.2 Tree Initialization

```math
T=(V,E)
```

```math
V=\{\chi_{start}\}
```

```math
E=\varnothing
```

---

## 2.3 Random Sampling

```math
x_{rand}\sim \mathrm{Uniform}(\chi)
```

---

## 2.4 Nearest Node Search

```math
x_{near}
=
\arg\min_{x\in V}
\left\|x-x_{rand}\right\|
```

---

## 2.5 Node Expansion Distance

```math
d
=
\left\|x_{rand}-x_{near}\right\|
```

---

## 2.6 New Node Generation

```math
x_{new}
=
\begin{cases}
x_{rand}, & d<\eta \\[4pt]
x_{near}
+
\dfrac{x_{rand}-x_{near}}{d}\eta,
& \mathrm{otherwise}
\end{cases}
```

---

## 2.7 Collision-Free Condition

```math
\mathrm{ObstacleFree}(x_{near},x_{new})
=
\begin{cases}
\mathrm{True}, & \mathrm{if\ no\ collision\ occurs} \\[4pt]
\mathrm{False}, & \mathrm{if\ collision\ occurs}
\end{cases}
```

---

## 2.8 Neighbor Set

```math
X_{near}
=
\left\{
x\in V
\mid
\left\|x-x_{new}\right\|<r
\right\}
```

---

## 2.9 Search Radius

```math
r
=
\gamma
\left(
\frac{\log |V|}{|V|}
\right)^{1/d_s}
```

where:

- `gamma` is a constant.
- `d_s` is the dimension of the state space.

---

## 2.10 Parent Selection

```math
x_{min}
=
\arg\min_{x\in X_{near}}
\left(
\mathrm{Cost}(x)
+
\left\|x-x_{new}\right\|
\right)
```

---

## 2.11 Path Cost

```math
\mathrm{Cost}(x)
=
\sum_{i=1}^{n}
\left\|x_i-x_{i-1}\right\|
```

---

## 2.12 Tree Update

```math
V
=
V\cup \{x_{new}\}
```

```math
E
=
E\cup \{(x_{min},x_{new})\}
```

---

## 2.13 Rewiring Condition

```math
\mathrm{Cost}(x_{new})
+
\left\|x_{new}-x\right\|
<
\mathrm{Cost}(x)
```

If this condition is satisfied:

```math
\mathrm{Parent}(x)=x_{new}
```

```math
\mathrm{Cost}(x)
=
\mathrm{Cost}(x_{new})
+
\left\|x_{new}-x\right\|
```

---

# 3. OBB and SAT Collision Detection

## 3.1 OBB Representation

```math
C=(c_x,c_y,c_z)
```

```math
H=(h_x,h_y,h_z)
```

```math
R=[u,v,w]
```

For obstacle `i`:

```math
R_i=[u_i,v_i,w_i]
```

---

## 3.2 Projection Interval

```math
\mathrm{Projection}_i
=
\left[
C_i\cdot L-r_i,
\;
C_i\cdot L+r_i
\right]
```

---

## 3.3 Projection Radius

```math
r_i
=
h_{x_i}|u_i\cdot L|
+
h_{y_i}|v_i\cdot L|
+
h_{z_i}|w_i\cdot L|
```

---

## 3.4 SAT Non-Intersection Condition

Two OBBs do not intersect if either condition holds:

```math
C_i\cdot L+r_i
<
C_j\cdot L-r_j
```

or:

```math
C_j\cdot L+r_j
<
C_i\cdot L-r_i
```

---

# 4. Path Optimization and Gait Assignment

## 4.1 Vertical Path Optimization Objective

```math
\min
\sum_{i=1}^{n-1}
|\Delta y_i|
```

---

## 4.2 Rolling Segment Constraint

```math
y_{i+1}^{new}
=
y_i^{new}
```

---

## 4.3 Vertical Increment Constraint

```math
y_{i+1}^{new}
=
y_i^{new}
+
\Delta y_i
```

```math
|\Delta y_i|
\le
\Delta y_{max}
```

---

## 4.4 Collision-Free Path Constraint

```math
\mathrm{ObstacleFree}
\left(
x_{i+1}^{new},
x_i^{new}
\right)
=
\mathrm{True}
```

---

## 4.5 Updated Node Position

```math
y_{i+1}^{new}
=
y_i
+
\sum_{j=1}^{i-1}
\Delta y_j^\ast
```

---

## 4.6 New Path Cost

```math
\mathrm{Cost}_{new}
=
\sum_{i=0}^{n-1}
\left\|
x_{i+1}^{new}
-
x_i^{new}
\right\|
```

---

## 4.7 Iterative Optimization Condition

```math
\exists i:
\neg
\mathrm{ObstacleFree}
\left(
x_{i+1}^{new},
x_i^{new}
\right)
\Rightarrow
\left|
y_{i+1}^{new}
-
y_i^{new}
\right|
\le
\epsilon
```

---

# 5. State-Space Model and EKF

## 5.1 Linear State-Space Model

```math
x_{k+1}
=
A x_k
+
B u_k
+
w_k
```

```math
y_k
=
C x_k
+
v_k
```

where:

- `x_k` is the state.
- `u_k` is the control input.
- `y_k` is the measurement.
- `w_k` is process noise.
- `v_k` is measurement noise.

---

## 5.2 State Vector

```math
x_k
=
\begin{bmatrix}
V_x \\
V_y \\
\varphi
\end{bmatrix}
```

```math
\varphi
=
\arctan
\left(
\frac{V_x}{V_y}
\right)
```

---

## 5.3 EKF Prediction

```math
\hat{x}_{k|k-1}
=
A_k
\hat{x}_{k-1|k-1}
+
B_k u_k
```

---

## 5.4 Covariance Prediction

```math
P_{k|k-1}
=
A_k
P_{k-1|k-1}
A_k^T
+
Q_k
```

---

## 5.5 Standard Kalman Gain

```math
K_k
=
P_{k|k-1}
C_k^T
\left(
C_k
P_{k|k-1}
C_k^T
+
R_k
\right)^{-1}
```

---

## 5.6 State Update

```math
\hat{x}_{k|k}
=
\hat{x}_{k|k-1}
+
K_k
\left(
y_k
-
C_k\hat{x}_{k|k-1}
\right)
```

---

## 5.7 Standard Covariance Update

```math
P_{k|k}
=
(I-K_kC_k)
P_{k|k-1}
```

---

# 6. Correlated-Noise EKF

## 6.1 Standard Assumption

```math
\mathrm{Cov}(w_k,v_k)=0
```

---

## 6.2 Correlated-Noise Assumption

```math
\mathrm{Cov}(w_k,v_k)\neq 0
```

The cross-covariance term is:

```math
S_k
```

---

## 6.3 Correlated-Noise Kalman Gain

```math
K_k
=
\left(
P_{k|k-1}C_k^T
+
S_k
\right)
\left(
C_kP_{k|k-1}C_k^T
+
R_k
+
H_kS_k
+
S_k^TH_k^T
\right)^{-1}
```

---

## 6.4 Correlated-Noise Covariance Update

```math
P_{k|k}
=
(I-K_kC_k)
P_{k|k-1}
(I-K_kC_k)^T
+
K_kR_kK_k^T
-
K_kS_k^T
-
S_kK_k^T
```

---

## 6.5 Bayesian Filtering Interpretation

```math
p(x_k\mid y_{1:k})
```

```math
p(x_k\mid y_{1:k})
\propto
p(y_k\mid x_k)
p(x_k\mid y_{1:k-1})
```

---

# 7. Lie Group Formulation

## 7.1 Pose on SE(2)

```math
X
=
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
\in
SE(2)
```

---

## 7.2 Lie Algebra Basis

```math
\hat{e}_1
=
\begin{bmatrix}
0 & 0 & 1 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
```

```math
\hat{e}_2
=
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 1 \\
0 & 0 & 0
\end{bmatrix}
```

```math
\hat{e}_3
=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
```

---

## 7.3 Lie Algebra Element

```math
\hat{\xi}
=
v_x\hat{e}_1
+
v_y\hat{e}_2
+
\omega\hat{e}_3
\in
\mathfrak{se}(2)
```

---

## 7.4 General Exponential Map Update

```math
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}
\right)
```

---

# 8. Swimming Dynamics

## 8.1 Continuous-Time Swimming Dynamics

```math
\dot{x}
=
v_y\tan\alpha\cos\theta
-
v_y\sin\theta
```

```math
\dot{y}
=
v_y\tan\alpha\sin\theta
+
v_y\cos\theta
```

```math
\dot{\theta}
=
0
```

---

## 8.2 Normalized Discrete-Time Swimming Dynamics

```math
x_{k+1}
=
x_k
+
\Delta t
\left(
\tan\alpha\cos\theta_k
-
\sin\theta_k
\right)
```

```math
y_{k+1}
=
y_k
+
\Delta t
\left(
\tan\alpha\sin\theta_k
-
\cos\theta_k
\right)
```

```math
\theta_{k+1}
=
\theta_k
```

---

## 8.3 Dimensional Discrete-Time Swimming Dynamics

```math
x_{k+1}
=
x_k
+
\Delta t
\left(
v_y\tan\alpha\cos\theta_k
-
v_y\sin\theta_k
\right)
```

```math
y_{k+1}
=
y_k
+
\Delta t
\left(
v_y\tan\alpha\sin\theta_k
+
v_y\cos\theta_k
\right)
```

```math
\theta_{k+1}
=
\theta_k
```

---

## 8.4 Swimming Body Velocity

```math
\xi_{swim}
=
\begin{bmatrix}
v_x \\
v_y \\
0
\end{bmatrix}
=
\begin{bmatrix}
v_y\tan\alpha \\
v_y \\
0
\end{bmatrix}
```

---

## 8.5 Swimming Lie Algebra Element

```math
\hat{\xi}_{swim}
=
\begin{bmatrix}
0 & 0 & v_y\tan\alpha \\
0 & 0 & v_y \\
0 & 0 & 0
\end{bmatrix}
\in
\mathfrak{se}(2)
```

---

## 8.6 Swimming Exponential Map

```math
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_{swim}
\right)
```

Expanded form:

```math
X_{k+1}
=
X_k
\begin{bmatrix}
1 & 0 & \Delta t\,v_y\tan\alpha \\
0 & 1 & \Delta t\,v_y \\
0 & 0 & 1
\end{bmatrix}
```

---

# 9. Rolling Dynamics

## 9.1 Continuous-Time Rolling Dynamics

```math
\dot{x}
=
\mu_r\omega_r
```

```math
\dot{y}
=
0
```

```math
\dot{\theta}
=
\omega_r
```

---

## 9.2 Discrete-Time Rolling Dynamics

```math
x_{k+1}
=
x_k
+
\Delta t\,\mu_r\omega_r
```

```math
y_{k+1}
=
y_k
```

```math
\theta_{k+1}
=
\theta_k
+
\Delta t\,\omega_r
```

---

## 9.3 Rolling Body Velocity

```math
\xi_{roll}
=
\begin{bmatrix}
\mu_r\omega_r \\
0 \\
\omega_r
\end{bmatrix}
```

---

## 9.4 Rolling Lie Algebra Element

```math
\hat{\xi}_{roll}
=
\begin{bmatrix}
0 & -\omega_r & \mu_r\omega_r \\
\omega_r & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\in
\mathfrak{se}(2)
```

---

## 9.5 Rolling Exponential Map

```math
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_{roll}
\right)
```

Expanded form:

```math
X_{k+1}
=
X_k
\begin{bmatrix}
\cos(\omega_r\Delta t)
&
-\sin(\omega_r\Delta t)
&
\mu_r\omega_r
\\
\sin(\omega_r\Delta t)
&
\cos(\omega_r\Delta t)
&
0
\\
0 & 0 & 1
\end{bmatrix}
```

---

# 10. Geometric Tracking Error

## 10.1 Relative Pose

```math
X_{rel}
=
X_k^{-1}
X_k^{ref}
```

```math
X_{rel}
=
\begin{bmatrix}
R_{rel} & t_{rel} \\
0 & 1
\end{bmatrix}
```

---

## 10.2 Rotation Error

```math
\omega
=
\operatorname{atan2}
\left(
R_{rel,21},
R_{rel,11}
\right)
```

---

## 10.3 Translation Error

```math
t_{rel}
=
\frac{\sin\omega}{\omega}p
+
\frac{1-\cos\omega}{\omega^2}
p^\perp
```

```math
p=
\begin{bmatrix}
v_x \\
v_y
\end{bmatrix}
```

```math
p^\perp
=
\begin{bmatrix}
-v_y \\
v_x
\end{bmatrix}
```

---

## 10.4 Error Vector

```math
e_k
=
\log
\left(
X_{rel}
\right)^\vee
=
\begin{bmatrix}
t_{rel,x} \\
t_{rel,y} \\
\omega
\end{bmatrix}
\in
\mathbb{R}^3
```

---

# 11. Model Predictive Control

## 11.1 MPC Objective

```math
J
=
\sum_{k=0}^{N-1}
\left(
e_k^TQe_k
+
u_k^TRu_k
\right)
+
e_N^TPe_N
```

---

## 11.2 Weighting Matrices

```math
Q
=
\operatorname{diag}
(q_x,q_y,q_\theta)
\succ 0
```

```math
R
=
\operatorname{diag}
(r_x,r_y)
\succ 0
```

```math
P
\succ 0
```

---

## 11.3 Dynamic Constraint

```math
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_k
\right)
```

---

## 11.4 Input Constraints

```math
v_y^{min}
\le
v_y
\le
v_y^{max}
```

```math
|\alpha|
\le
\alpha_{max}
```

---

## 11.5 Convexity Conditions

```math
\nabla_e^2
\left(
e^TQe
\right)
\succ
0
```

```math
\nabla_u^2
\left(
u^TRu
\right)
\succ
0
```

```math
\nabla_{e_N}^2
\left(
e_N^TPe_N
\right)
\succ
0
```

---

## 11.6 Jacobian of Control-to-Velocity Map

```math
J_k^{(i)}
=
\frac{\partial \xi_k}{\partial u_k}
=
\begin{bmatrix}
\dfrac{\partial v_x}{\partial v_y}
&
\dfrac{\partial v_x}{\partial \alpha}
\\
\dfrac{\partial v_y}{\partial v_y}
&
\dfrac{\partial v_y}{\partial \alpha}
\\
0 & 0
\end{bmatrix}
=
\begin{bmatrix}
\tan\alpha & v_y\sec^2\alpha \\
1 & 0 \\
0 & 0
\end{bmatrix}
```

---

## 11.7 Linearized Dynamics

```math
\xi_k
\approx
\xi_k^{(i)}
+
J_k^{(i)}
\left(
u_k-u_k^{(i)}
\right)
```

---

## 11.8 KKT Conditions

```math
\begin{cases}
\nabla_u J
+
\sum_i \lambda_i \nabla_u g_i
+
\sum_j \mu_j \nabla_u h_j
=
0
\\
g_i(u)\le 0,\quad \lambda_i\ge 0,\quad \lambda_i g_i(u)=0
\\
h_j(u)=0
\end{cases}
```

---

# 12. PID Rolling Control

## 12.1 PID Control Law

```math
\omega_r
=
K_p e_\theta
+
K_i
\int e_\theta dt
+
K_d
\frac{de_\theta}{dt}
+
K_{p,x}e_x
```

---

# 13. Hybrid Switching and Stability

## 13.1 Switching Signal

```math
\sigma(t)
\in
\{swim,roll\}
```

---

## 13.2 Hybrid System Representation

```math
\mathcal{H}
=
(\mathcal{C},F,\mathcal{D},G)
```

where:

- `C` is the flow set.
- `F` is the continuous dynamics.
- `D` is the jump set.
- `G` is the jump map.

---

## 13.3 Average Dwell Time Condition

```math
\tau_d
>
\tau_d^\star
```

A common switched-system bound is:

```math
N_\sigma(t_0,t)
\le
N_0
+
\frac{t-t_0}{\tau_d}
```

---

## 13.4 Lyapunov Decrease Condition

For mode `i`:

```math
\dot{V}_i(x)
\le
-\lambda_i V_i(x)
```

At switching instants:

```math
V_j(x)
\le
\mu V_i(x)
```

A sufficient dwell-time condition is:

```math
\tau_d
>
\frac{\ln\mu}{\lambda}
```

---

# 14. Soft-Robot Deformation and Propulsion Factor

## 14.1 Magnetic Torque

```math
\tau_m
=
VMB\sin\psi
```

---

## 14.2 Quasi-Static Deformation Approximation

```math
\dot{\varepsilon}
\gg
\dot{v}
```

```math
\varepsilon(t)
\approx
\frac{
VMB\sin\psi
}{
EI/L+\gamma\eta\omega
}
```

---

## 14.3 Propulsive Force

```math
F_{push}
=
\alpha(\varepsilon)f
```

---

## 14.4 Velocity-Input Relation

```math
v(t)
=
\alpha(\varepsilon(t))u(t)
```

---

## 14.5 Propulsion Efficiency With Energy Dissipation

```math
\alpha(\varepsilon)
=
\alpha_0
\left(
1-\beta\varepsilon^2
\right)
```

---

## 14.6 High-Dimensional Deformation-Aware Dynamics

```math
\dot{x}
=
f(x,u,\varepsilon)
```

```math
\dot{\varepsilon}
=
g(u,\varepsilon)
```

---

## 14.7 Nominal Linear Model

```math
x_{k+1}
=
A x_k
+
B u_k
```

```math
B
=
\begin{bmatrix}
\overline{\alpha(\varepsilon)}\Delta t \\
0
\end{bmatrix}
```

---

## 14.8 Deformation-Dependent System

```math
x_{k+1}
=
A(\varepsilon)x_k
+
B(\varepsilon)u_k
```

---

## 14.9 MPC Nominal Prediction

```math
\hat{x}_{k+i|k}
=
A_0^i x_k
+
\sum_{j=0}^{i-1}
A_0^{i-1-j}
B_0u_{k+j}
```

---

## 14.10 Actual Deformation-Dependent Trajectory

```math
x_{k+i}
=
\left(
\prod_{j=0}^{i-1}
A(\varepsilon_{k+j})
\right)
x_k
+
\sum_{j=0}^{i-1}
\left(
\prod_{l=j+1}^{i-1}
A(\varepsilon_{k+l})
\right)
B(\varepsilon_{k+j})
u_{k+j}
```

---

## 14.11 Prediction Error

```math
e_{k+i}
=
x_{k+i}
-
\hat{x}_{k+i|k}
```

---

## 14.12 First-Order Error Bound

```math
\left\|
e_{k+i}
\right\|
\le
\sum_{j=0}^{i-1}
\left\|
A_0^{i-1-j}
\right\|
\left(
\left\|A_1\right\|
\left\|x\right\|
+
\left\|B_1\right\|
\left\|u\right\|
\right)
\left\|
\varepsilon_{k+i}
\right\|
```

---

## 14.13 Deformation Influence in PID Control

```math
e
=
v_{ref}
-
\alpha(\varepsilon)u
```

---

# 15. Physics-Informed Learning Extension

This section is a future extension aligned with physics-informed learning, dynamical systems, and control.

## 15.1 Physics-Informed Loss

```math
\mathcal{L}
=
\mathcal{L}_{data}
+
\lambda_p\mathcal{L}_{physics}
+
\lambda_b\mathcal{L}_{boundary}
+
\lambda_c\mathcal{L}_{control}
```

---

## 15.2 PDE-Constrained Dynamics

```math
\frac{\partial z}{\partial t}
+
\mathcal{N}(z)
=
0
```

---

## 15.3 Learned Residual Dynamics

```math
x_{k+1}
=
f_{nominal}(x_k,u_k)
+
f_\theta(x_k,u_k)
```

---

## 15.4 Physics-Regularized Control Objective

```math
\min_{\theta,u}
\mathcal{L}_{data}
+
\lambda_p
\left\|
\frac{\partial z}{\partial t}
+
\mathcal{N}(z)
\right\|^2
+
\lambda_u
\sum_{k=0}^{N-1}
\left\|
u_k
\right\|^2
```

---

# End of Formula File
