# Floating-Base Robotic Arm Dynamics in Microgravity

A full **physics-based simulation of a free-floating multi-DOF robotic arm in space**, developed in MATLAB.

This project models how robotic manipulators behave in **microgravity**, capturing:
- Base reaction motion
- Momentum conservation
- Center of mass drift
- Joint-space dynamics

---

## 🛰️ Project Motivation
In space, robotic arms do not have a fixed base.  
Any joint motion induces **reaction forces and torques on the base**, making dynamics fundamentally different from terrestrial robots.

This project explores those effects using:
- Lagrangian dynamics
- Conservation of linear & angular momentum
- URDF-based robot modeling

---

## ⚙️ System Overview
- **Robot:** 5-DOF arm + floating base (6 links total)
- **Environment:** Microgravity (no external forces)
- **Modeling:** Full rigid-body dynamics
- **Input:** Manual or randomized joint trajectories
- **Output:** Joint states, base reactions, COM evolution

---

## 🧠 Key Features
- ✔️ Floating-base dynamics (no fixed reference)
- ✔️ Inertia tensors computed from STL geometry
- ✔️ URDF-based kinematic structure
- ✔️ Center of mass tracking in inertial frame
- ✔️ Reaction torque computation using momentum conservation
- ✔️ ODE-based dynamic simulation (`ode45`)
- ✔️ Energy & momentum validation plots
- ✔️ CSV export for post-processing

---

## 📊 Outputs & Visuals
- Joint angles, velocities, accelerations
- Base angular velocity and reaction torque
- Total kinetic energy
- Global center of mass trajectory
- 2D/3D animation of robot motion

---

## 🛠️ Tools Used
- MATLAB
- Robotics System Toolbox
- Symbolic Math Toolbox
- STL mesh processing

---

## 📚 References
- Ranjan Vepa, *Dynamics and Control of Autonomous Space Vehicles and Robotics*
- Classical rigid-body dynamics formulations

---

## 📌 Why This Project Matters
This project demonstrates:
- Strong understanding of **advanced robot dynamics**
- Ability to implement **theory-heavy models**
- Careful validation using physical laws
- Consistent iteration toward realistic behavior

