---
id: hw-safety-protocols
title: "Safety Protocols & Best Practices"
sidebar_position: 4
sidebar_label: "Safety Protocols"
description: "Best practices for safe robot operation, testing, and deployment"
keywords: [safety, protocols, emergency, risk mitigation, best practices]
---

## Overview

This chapter provides **safety protocols and best practices** for all robotics work in this course, whether simulation-only or with hardware. Safety is paramount when working with autonomous systems.

---

## Core Safety Principles

### 1. Assume Everything Will Fail Unexpectedly

- Network connections drop
- Sensor readings become corrupted
- Motors respond unpredictably
- Humans appear in robot's workspace

**Mitigation**: Always have a kill switch and operator supervision.

### 2. Default to Safe State

When in doubt, the robot should stop moving rather than continue.

```python
# Bad: Keep moving until success
while not goal_reached:
    move_robot()

# Good: Stop if anything unexpected happens
try:
    while not goal_reached:
        move_robot()
except (RobotException, TimeoutError, NetworkError):
    STOP_ALL_MOTION()
    LOG_ERROR()
```

### 3. Layers of Defense

Never rely on a single safety mechanism:

1. **Software level**: Velocity limits, joint constraints
2. **Hardware level**: Physical stops, breakers
3. **Environmental**: Barriers, emergency stop button
4. **Operator**: Human supervision and intervention capability

---

## Pre-Operation Safety Checklist

Use this checklist **every time** before running robots:

### Environmental Safety

- [ ] **Clear Space**: Testing area is empty of people, pets, obstacles
- [ ] **Barriers**: Physical boundaries or flags marking robot workspace
- [ ] **Emergency Space**: At least 2 meters of clear space in all directions
- [ ] **Surface**: Level ground; no holes, wet spots, or hazards
- [ ] **Lighting**: Area is well-lit for operator visibility
- [ ] **Exits**: Clear escape routes in case of emergency

### Equipment Verification

- [ ] **Power Supply**: Battery is fully charged; cables are intact
- [ ] **Actuators**: Motors respond to test commands; no grinding noises
- [ ] **Sensors**: Camera, LiDAR, IMU all reporting valid data
- [ ] **Network**: ROS 2 nodes are connected; topics publishing normally
- [ ] **E-Stop**: Emergency stop button is functional and reachable
- [ ] **Tether** (if used): Safety line is attached and not tangled

### Software Readiness

- [ ] **Safety Limits**: Velocity/torque limits are properly configured
- [ ] **Watchdog**: Network timeout detection is enabled
- [ ] **Logging**: All operations will be logged for review
- [ ] **Constraints**: Joint limits and collision checks are active
- [ ] **Test Mode**: Running in reduced-speed or test mode if first time
- [ ] **Operator**: Trained supervisor is present and monitoring

### Documentation

- [ ] **Start Time**: Logged (for incident investigation if needed)
- [ ] **Operator Name**: Documented
- [ ] **Objectives**: What the test is trying to accomplish
- [ ] **Baseline Behavior**: Expected normal behavior recorded

---

## Emergency Procedures

### If Robot Starts Behaving Unexpectedly

1. **Immediate Action** (< 1 second):
   - Press E-stop button
   - Or unplug power if E-stop fails
   - Move away from robot

2. **Assessment** (next 5 seconds):
   - Is anyone injured?
   - Is equipment damaged?
   - Is the area safe?

3. **Investigation** (after immediate danger is clear):
   - Capture logs and state information
   - Don't modify anything (preserve evidence)
   - Document what happened: time, behavior, circumstances

4. **Resolution**:
   - Verify safety systems before restarting
   - Address root cause (software, hardware, environment)
   - Run diagnostic tests before resuming normal operation

### If Person Enters Robot Workspace

1. **Robot operator**:
   - Immediately press E-stop
   - Clearly communicate "Stop! Danger!"

2. **Bystander**:
   - Move away from robot
   - Alert operator if robot is still moving

3. **After**:
   - Verify no injuries
   - Investigate what allowed person to enter (zone awareness failed?)
   - Strengthen barrier or monitoring

### If Network Connection Is Lost

**Automatic behavior** (implemented in firmware):
```python
# ROS 2 watchdog timer
def watchdog_callback():
    """Called if no heartbeat from operator for 2 seconds."""
    STOP_ALL_MOTION()
    ENGAGE_BRAKES()
    SOUND_ALARM()
    LOG_DISCONNECTION()

# Timer is reset every time valid command received
def receive_command(cmd):
    watchdog_timer.reset()
    execute_command(cmd)
```

**Do not**: Implement "continues last command" on network loss

---

## Software Safety Patterns

### Velocity Saturation

Always limit maximum velocity before sending to robot:

```python
from geometry_msgs.msg import Twist

def apply_velocity_limits(cmd_vel, limits):
    """Clip velocity commands to safe limits."""
    cmd_vel.linear.x = max(limits.linear_x_min,
                           min(limits.linear_x_max, cmd_vel.linear.x))
    cmd_vel.linear.y = max(limits.linear_y_min,
                           min(limits.linear_y_max, cmd_vel.linear.y))
    cmd_vel.angular.z = max(limits.angular_z_min,
                            min(limits.angular_z_max, cmd_vel.angular.z))
    return cmd_vel

# Example
limits = {
    'linear_x_max': 0.5,   # m/s
    'linear_x_min': -0.3,
    'linear_y_max': 0.2,
    'angular_z_max': 0.3   # rad/s
}

safe_velocity = apply_velocity_limits(user_command, limits)
robot.publish(safe_velocity)
```

### Joint Limit Enforcement

```python
def check_joint_limits(target_angles, min_angles, max_angles):
    """Verify joint angles are within limits."""
    for i, (target, min_val, max_val) in enumerate(
        zip(target_angles, min_angles, max_angles)):
        if target < min_val or target > max_val:
            raise JointLimitError(
                f"Joint {i}: target {target} outside [{min_val}, {max_val}]"
            )
    return True

# Usage
try:
    check_joint_limits(ik_solution, joint_mins, joint_maxs)
    publish_joint_commands(ik_solution)
except JointLimitError as e:
    print(f"Safety violation: {e}")
    STOP_ALL_MOTION()
```

### Timeout Detection

```python
import time
from threading import Timer

class CommandWatchdog:
    """Ensure commands are fresh; stop robot if timeout."""

    def __init__(self, timeout_sec=0.5):
        self.timeout_sec = timeout_sec
        self.last_command_time = time.time()
        self.is_alive = True

    def update_command(self):
        """Called when valid command received."""
        self.last_command_time = time.time()

    def check_freshness(self):
        """Return True if command is fresh (< timeout)."""
        age = time.time() - self.last_command_time
        if age > self.timeout_sec:
            self.is_alive = False
            STOP_ALL_MOTION()
            return False
        return True

# In main loop
watchdog = CommandWatchdog(timeout_sec=1.0)

while running:
    if not watchdog.check_freshness():
        # Command is stale; robot already stopped
        break
```

### Collision Avoidance

```python
def check_collision_risk(target_velocity, sensor_data, min_safe_distance=0.5):
    """Warn or stop if obstacles detected."""
    obstacle_distance = sensor_data.get_closest_obstacle()

    if obstacle_distance < min_safe_distance:
        if obstacle_distance < min_safe_distance * 0.5:
            # Emergency stop: very close obstacle
            return Twist()  # Zero velocity
        elif target_velocity.linear.x > 0:
            # Reduce forward velocity if approaching obstacle
            reduction_factor = obstacle_distance / min_safe_distance
            target_velocity.linear.x *= reduction_factor

    return target_velocity
```

---

## Hardware Safety Features

### Emergency Stop (E-Stop) Button

**Requirements**:
- [ ] Large, red, mushroom-shaped button
- [ ] Immediately accessible to operator
- [ ] Cuts power to motors (not just sending stop command)
- [ ] Tested weekly to ensure functionality
- [ ] Clearly labeled "EMERGENCY STOP"

**Testing E-Stop**:
```bash
# Before each operation session
1. Press E-Stop
2. Verify robot immediately stops all motion
3. Verify cannot restart without resetting E-Stop
4. Release E-Stop to reset
5. Verify normal operation resumes
```

### Motor Brakes

Ensure robots that could roll/slide have mechanical brakes:

```python
class RobotWithBrakes:
    """Robot that engages brakes during failure."""

    def shutdown(self):
        """Graceful shutdown: brakes on, power off."""
        self.disable_motors()
        self.engage_brakes()  # Mechanical brake
        self.power_off()
```

### Power Supply Protection

- [ ] Fuse or breaker rated for motor current
- [ ] Protection from short circuits
- [ ] Battery disconnect switch easily accessible
- [ ] No exposed high-voltage connections

---

## Testing & Validation

### Incremental Testing Strategy

**Never** test all features simultaneously. Build up gradually:

**Stage 1: Stationary Operation** (Robot not moving)
- Sensors read correctly
- Software initializes without errors
- Commands are received without crashes

**Stage 2: Slow Motion** (< 0.1 m/s)
- Robot moves in controlled direction
- Can stop immediately if needed
- Acceleration is smooth (no jerking)

**Stage 3: Normal Speed** (0.1–0.5 m/s)
- Motion is predictable
- Obstacle detection works
- Network latency is acceptable

**Stage 4: Full Autonomy** (voice commands, full speed)
- Only if all previous stages passed
- Operator stands by with E-stop
- Video recording enabled for review

### Testing Checklist

Each stage, verify:
- [ ] Robot starts from safe state (powered off)
- [ ] All sensors initialized correctly
- [ ] Network ping time is < 50 ms
- [ ] Velocity/torque limits are active
- [ ] Watchdog timer is running
- [ ] Operator ready with E-stop
- [ ] Clear path to testing goal
- [ ] Logging is recording all commands
- [ ] Robot stops when expected
- [ ] No unexpected behavior

### Failure Analysis Log

Whenever something unexpected happens:

```markdown
## Incident Report: [Date] [Time]

**What happened**: [Description of unexpected behavior]

**When it happened**: [Timestamp from logs]

**Who was operating**: [Operator name]

**Conditions at time**:
- Battery voltage: X V
- Network latency: X ms
- Robot position: X, Y
- Temperature: X°C

**What triggered it**: [Best guess of root cause]

**How it was resolved**: [What action stopped the problem]

**Root cause analysis**: [After investigation]

**Preventive measures**: [Changes made to prevent recurrence]

**Follow-up**: [Who needs to review; remediation plan]
```

---

## Standard Operating Procedures (SOPs)

### SOP 1: Starting the Robot

```
1. Visual inspection
   - No visible damage
   - No loose components
   - Battery connected

2. Power-on sequence
   - Flip main power switch
   - Wait 5 seconds for boot
   - Check power LED is green

3. ROS 2 initialization
   - Source ROS 2 setup: source /opt/ros/humble/setup.bash
   - Launch robot driver: ros2 launch my_robot robot.launch.py
   - Verify topics: ros2 topic list (should show /cmd_vel, /sensor_data, etc.)

4. Sensor check
   - Camera: ros2 topic echo /camera/image_raw | head -5
   - LiDAR: ros2 topic echo /scan | head -5
   - IMU: ros2 topic echo /imu | head -5

5. Safety systems check
   - E-Stop: Press and verify stop
   - Watchdog: Unplug network cable 2 sec; verify robot stops
   - Limits: Send max velocity command; verify clamped to limit

6. Ready for operation
   - All checks passed
   - Operator is present
   - Area is clear
   - Logging is active
```

### SOP 2: Running Autonomous Code

```
1. Pre-flight
   - Follow SOP 1 (robot startup)
   - Verify test objectives with supervisor
   - Record start time and operator name in log

2. Execution
   - Operator stands by with E-stop hand on button
   - Send initial motion command (low speed)
   - Monitor for 10 seconds (expected behavior?)
   - If OK, increase complexity/speed incrementally
   - Never leave robot unattended while autonomous

3. Termination
   - Send stop command
   - Verify robot halts
   - Review logs for errors
   - Document observations

4. Post-operation
   - Shut down robot: ros2 lifecycle set /robot_node shutdown
   - Disconnect power
   - Secure area
   - Review logs and file incident reports if needed
```

### SOP 3: Shutting Down

```
1. Command robot to stop
   - Send zero velocity: ros2 topic pub /cmd_vel geometry_msgs/Twist "{}"

2. Shut down ROS 2 gracefully
   - Press Ctrl+C in terminal running robot driver
   - Wait 2 seconds for cleanup

3. Disable power
   - Flip main power switch OFF
   - Verify all LEDs extinguish

4. Final checks
   - Robot should not move (motors de-energized)
   - No beeping or error sounds
   - Temperature is reasonable (not hot)

5. Log shutdown
   - Record stop time
   - Note any issues observed
   - File any incident reports
```

---

## Risk Assessment Matrix

| Hazard | Probability | Severity | Mitigation |
| --- | --- | --- | --- |
| Robot collision with person | Low* | High | E-Stop, barriers, supervisor, sensors |
| Network timeout | Medium | Medium | Watchdog timer, hardwired E-stop |
| Unexpected acceleration | Low | Medium | Velocity limits, gradual startup |
| Sensor failure (camera/LiDAR) | Low | Low | Graceful degradation, fallback behavior |
| Power loss mid-motion | Very Low | Medium | Motor brakes, safe state default |
| Code bug / infinite loop | Low | Medium | Watchdog timer, code review, testing |
| Environmental hazard (objects) | Medium | Low | Clear testing area, operator awareness |

*Assuming proper barriers and supervision

---

## Training & Certification

### Operator Certification Levels

**Level 1: Simulator Only**
- ✅ Understand ROS 2 basics
- ✅ Comfortable with simulation tools (Gazebo, Isaac)
- ✅ No hardware interaction
- ✅ Requirements: Course completion

**Level 2: Hardware (Stationary)**
- ✅ Pass Level 1
- ✅ Understand safety protocols
- ✅ Can operate stationary sensors (cameras, microphones)
- ✅ Requirements: Safety training + supervised sessions

**Level 3: Hardware (Autonomous Motion)**
- ✅ Pass Level 2
- ✅ Demonstrated vehicle control in simulation
- ✅ Understand emergency procedures
- ✅ Requirements: 10 supervised sessions + signed safety acknowledgment

**Level 4: Advanced (Multi-Sensor/VLA)**
- ✅ Pass Level 3
- ✅ Understanding of sensor fusion and autonomous decision-making
- ✅ Requirements: Instructor approval + published safety review

### Safety Acknowledgment Form

Every operator must sign before first hardware use:

```
--- ROBOT SAFETY ACKNOWLEDGMENT ---

I confirm that I have:
☐ Read the safety protocols in full
☐ Understood the emergency procedures
☐ Been trained on the E-stop button
☐ Reviewed the pre-operation checklist
☐ Understand the risks of autonomous robot operation
☐ Agree to follow all safety procedures strictly
☐ Understand that violation of safety rules may result in loss of access

I acknowledge that failure to follow safety protocols could result in
injury to persons or damage to equipment. I agree to take full
responsibility for my actions and to prioritize safety above all.

Operator Name: ___________________
Date: ___________________
Supervisor Witness: ___________________

This acknowledgment must be on file before operator uses hardware.
```

---

## Incident Response

### If an Accident Occurs

1. **Immediate** (first 30 seconds):
   - Stop the robot (E-Stop if necessary)
   - Check for injuries
   - Call emergency services if needed (911 in US)
   - Move people to safety

2. **Short-term** (next 30 minutes):
   - Do not restart robot
   - Do not modify or delete any logs/evidence
   - Document scene with photos
   - Collect witness statements

3. **Medium-term** (same day):
   - File incident report (template below)
   - Notify supervisor and institution safety officer
   - Secure all evidence (logs, video, physical artifacts)
   - Begin root cause analysis

4. **Long-term** (days/weeks):
   - Conduct formal investigation
   - Identify systemic issues (not just immediate cause)
   - Implement corrective actions
   - Brief all operators on lessons learned
   - Update procedures/training if needed

### Incident Report Template

```markdown
# Incident Report

**Date/Time**: [YYYY-MM-DD HH:MM:SS]
**Location**: [Lab/Testing Area]
**Operator**: [Name]
**Supervisor**: [Name]

## Summary
[One-sentence description of what happened]

## Detailed Description
[Chronological timeline of events leading to incident]

## Injuries/Damage
[List any injuries or equipment damage]

## Contributing Factors
[Environmental, software, hardware, human factors]

## Root Cause Analysis
[What was the underlying reason this happened?]

## Corrective Actions
[What will prevent this in future?]

## Timeline for Implementation
[When will fixes be in place?]

## Sign-Off
Report prepared by: [Name]
Reviewed by supervisor: [Name]
Approved by safety officer: [Name]
```

---

## References & Further Reading

- **OSHA Robot Safety Guidelines**: [https://www.osha.gov/robotics](https://www.osha.gov/robotics)
- **ISO/TS 15066:2016** (Collaborative Robots Safety): Industrial standard for human-robot interaction
- **ROS 2 Security**: [https://docs.ros.org/en/humble/Concepts/About-Security.html](https://docs.ros.org/en/humble/Concepts/About-Security.html)
- **Robotics Safety Survey**: Academic research on autonomous system safety

---

## Quick Reference: Safety Command Card

Print and post near testing area:

```
╔══════════════════════════════════════════════════════════════╗
║                  ROBOT SAFETY QUICK REFERENCE                ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  BEFORE START:  Check checklist ☐  Clear area ☐            ║
║                 Supervisor ready ☐  Logging on ☐            ║
║                                                              ║
║  EMERGENCY:     Press RED E-STOP button                     ║
║                 Do NOT try to catch robot                   ║
║                 Move away from impact zone                  ║
║                                                              ║
║  MAX SPEED:     Linear:  0.5 m/s                            ║
║                 Angular: 0.3 rad/s                          ║
║                                                              ║
║  NETWORK FAIL:  Robot stops automatically (1 sec timeout)   ║
║                                                              ║
║  OPERATOR CALL: [Supervisor Name] [Phone]                  ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
```

---

**Last Updated**: 2025-12-10
**Relevant For**: All modules; essential reading before any hardware use
**Capstone Connection**: Safety protocols are critical for voice-controlled robot capstone project
