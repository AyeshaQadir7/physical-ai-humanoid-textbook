---
id: module-2-sim-to-real
title: "Sim-to-Real Considerations"
sidebar_position: 5
sidebar_label: "Sim-to-Real"
description: "Bridging the reality gap and transferring simulated learning to physical robots"
keywords: [sim-to-real, reality gap, domain randomization, transfer learning, validation]
---

# Sim-to-Real Considerations

## Introduction

The biggest challenge in robotics: **simulation ≠ reality**.

This chapter covers:
- Understanding the reality gap
- Domain randomization for robustness
- Hardware validation
- Safety before deployment
- Transfer learning strategies

---

## Learning Outcomes

By the end, you will:
1. Understand why simulations differ from reality
2. Use domain randomization to improve transfer
3. Validate learned behaviors on hardware
4. Deploy safely to real robots
5. Quantify sim-to-real error

---

## Part 1: The Reality Gap

### What Differs Between Simulation and Reality?

| Factor | Simulation | Reality |
|--------|-----------|---------|
| **Friction** | Constant coefficient | Variable, depends on surface |
| **Inertia** | Perfect mass | Center of mass shifts with batteries, wear |
| **Sensor noise** | Gaussian (fake) | Non-Gaussian, biased, drifts over time |
| **Latency** | Zero | Network delay, processing delay |
| **Actuation** | Perfect response | Motor deadband, saturation, backlash |
| **Collisions** | Geometric | Material deformation, energy loss unpredictable |
| **Environment** | Controlled | Lighting, temperature, humidity, vibration |

### Example: Walking Robot

**In Gazebo**:
- Gravity: exactly 9.81 m/s²
- Friction: constant 0.7
- Leg inertia: symmetric
- → Robot walks smoothly

**In reality**:
- Gravity varies by location
- Friction changes with surface (tile, carpet, sand)
- Leg inertia changes as batteries discharge
- → Robot stumbles unpredictably

### Size of Reality Gap

| Skill | Simulation-to-Real Transfer Rate |
|-------|----------------------------------|
| Vision (object detection) | 75–90% (good transfer) |
| Grasping | 40–60% (moderate gap) |
| Locomotion | 50–70% (moderate gap) |
| Manipulation (pushing) | 30–50% (large gap) |

---

## Part 2: Domain Randomization

### Concept

**Domain randomization** adds variability to simulation to match real-world diversity:

```python
import numpy as np

def randomize_physics():
    """Randomize physics parameters for each episode"""
    friction = np.random.uniform(0.3, 1.0)  # Varied friction
    gravity = np.random.uniform(9.5, 10.0)  # Gravity varies by location
    mass_multiplier = np.random.uniform(0.9, 1.1)  # Mass variation

    # Apply to Gazebo
    set_gazebo_friction(friction)
    set_gazebo_gravity(gravity)
    set_gazebo_mass(mass_multiplier)
```

### What to Randomize

**Physics**:
- Friction coefficients (±30%)
- Mass (±20%)
- Gravity (±2%)
- Motor damping

**Sensors**:
- Camera noise (Gaussian + salt-and-pepper)
- LiDAR range noise
- IMU bias drift

**Environment**:
- Lighting (brightness, shadows)
- Object positions (±10cm)
- Friction textures (random surfaces)

### Example: Randomized Friction for Walking

```python
def train_walking_controller(episodes=1000):
    for episode in range(episodes):
        # Randomize friction each episode
        friction = np.random.uniform(0.3, 1.0)
        set_simulation_friction(friction)

        # Train policy
        policy.train_episode(env)

        # Collect data: (state, action, reward)

    return policy  # Robust to friction variation
```

**Result**: Policy trained on varied friction transfers better to real floors.

---

## Part 3: Validation on Hardware

### Progression: Simulation → Hardware

```
1. Develop in simulation (Gazebo)
   ├─ Test on diverse randomized environments
   ├─ Verify physics (gravity, friction, inertia)
   └─ Qualitative behavior matches expectations

2. Transfer to real hardware
   ├─ Start with simple task (pick up cube)
   ├─ Monitor closely (safety personnel nearby)
   ├─ Collect data (compare sim vs. real)
   └─ Measure success rate

3. If Under 80% success on hardware:
   ├─ Identify failure modes
   ├─ Add randomization to simulation
   ├─ Retrain policy
   └─ Go to step 2

4. Once >90% success:
   ├─ Increase task complexity
   ├─ Add sensors (camera, LiDAR)
   ├─ Test edge cases
   └─ Deploy to production
```

### Example Validation Script

```python
def validate_on_hardware(policy, num_trials=20):
    """Test policy on real robot"""
    successes = 0

    for trial in range(num_trials):
        # Reset robot to known state
        reset_robot()

        # Run policy
        state = get_state()
        for step in range(50):  # Max 50 steps
            action = policy.predict(state)
            state, reward, done = robot.step(action)
            if done:
                break

        # Check if successful (e.g., cube picked up)
        if check_success():
            successes += 1
            log_video(trial)  # Record successes
        else:
            log_video(trial, failure=True)  # Record failures for analysis

    success_rate = successes / num_trials
    print(f"Hardware success rate: {success_rate * 100:.1f}%")

    if success_rate > 0.9:
        print("Ready for deployment!")
    else:
        print("Needs more training with randomization")
```

---

## Part 4: Safety Validation

**Never deploy untested code to real robots!**

### Safety Checklist

- [ ] Emergency stop button works (kill power in Under 100ms)
- [ ] Velocity limits enforced (e.g., max 0.5 m/s)
- [ ] Torque limits enforced (prevent damage)
- [ ] Collision detection active (stop if hits obstacle)
- [ ] Battery monitoring (graceful shutdown if low)
- [ ] Thermal monitoring (shutdown if overheat)
- [ ] Network timeout (revert to safe state if disconnected)

### Example: Safe Motion Limits

```python
class SafeRobot:
    def __init__(self, max_velocity=0.5, max_torque=100):
        self.max_velocity = max_velocity
        self.max_torque = max_torque

    def execute_action(self, action):
        # Clamp velocity
        action = np.clip(action, -self.max_velocity, self.max_velocity)

        # Check temperature
        if self.get_temperature() > 80:  # Celsius
            self.shutdown("Overheat protection")
            return

        # Check battery
        if self.get_battery() < 5:  # Percent
            self.shutdown("Low battery")
            return

        # Send to hardware
        self.send_command(action)
```

---

## Part 5: Quantifying Sim-to-Real Error

### Metrics

```python
def measure_sim_to_real_error():
    """Compare simulated vs. real behavior"""

    # Run same trajectory in sim and reality
    trajectory_sim = run_simulation()
    trajectory_real = run_on_hardware()

    # Metrics
    l2_distance = np.linalg.norm(trajectory_sim - trajectory_real)
    success_rate_sim = count_successes(trajectory_sim)
    success_rate_real = count_successes(trajectory_real)
    gap = abs(success_rate_sim - success_rate_real)

    print(f"L2 distance: {l2_distance:.3f}")
    print(f"Success rate (sim): {success_rate_sim * 100:.1f}%")
    print(f"Success rate (real): {success_rate_real * 100:.1f}%")
    print(f"Sim-to-real gap: {gap * 100:.1f}%")
```

---

## Part 6: Real Robot Deployment Workflow

### Week-by-Week Progression

**Week 1**: Simple pick-and-place
- Train in Gazebo with domain randomization
- Test on hardware (1 trial per day)
- Collect failure data

**Week 2**: Add obstacle avoidance
- Integrate LiDAR perception
- Validate collision detection
- Test in cluttered environment

**Week 3**: Add voice control
- Integrate Whisper for speech-to-text
- Add language-to-action mapping
- Test with diverse voice inputs

**Week 4**: Full integration test
- Combine all modules
- Test on varied tasks
- Validate safety systems

### Deployment Checklist

- [ ] Simulation training complete (>95% success in sim)
- [ ] Domain randomization covers expected real variations
- [ ] Hardware validation: >80% success rate
- [ ] Safety systems tested and working
- [ ] Emergency stop verified
- [ ] Thermal, battery, timeout protection enabled
- [ ] Video recorded for documentation
- [ ] Team trained on safety protocols

---

## Summary

**Reality Gap**:
- Friction, inertia, sensor noise differ
- Transfer rate: 40–90% depending on task

**Domain Randomization**:
- Add variability to training
- Improves real-world robustness
- Standard approach in robotics ML

**Validation**:
- Test incrementally
- Measure success on hardware
- Validate safety before deployment

**Deployment**:
- Progression: simple → complex
- Monitor carefully
- Have emergency stop ready

---

## Navigation

- **Previous**: [Chapter 4: Unity](./04-unity-for-robotics.md)
- **Next**: [Module 2 Labs](./lab-2-1-load-robot-in-gazebo.md)
