---
id: module-2-unity
title: "Unity for Robotics"
sidebar_position: 4
sidebar_label: "Unity"
description: "High-fidelity visualization and human-robot interaction with Unity"
keywords: [Unity, visualization, rendering, HRI, robotics, simulation]
---

# Unity for Robotics

## Introduction

**Gazebo** is physics-accurate but visually basic. **Unity** offers stunning graphics and human-robot interaction (HRI) visualization.

This chapter covers:
- When to use Unity vs. Gazebo
- URDF import into Unity
- Photorealistic rendering
- ROS 2 integration

---

## Learning Outcomes

By the end, you will:
1. Understand Gazebo vs. Unity tradeoffs
2. Import URDF into Unity
3. Visualize robots with high fidelity
4. Use ROS 2 bridge in Unity
5. Know when each tool is appropriate

---

## Part 1: Gazebo vs. Unity

### Comparison

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Physics accuracy** | High | Medium |
| **Visual fidelity** | Low | Very High |
| **ROS 2 integration** | Native | Via ROS 2 bridge |
| **Performance** | Fast (headless) | Requires GPU |
| **Learning curve** | Moderate | Steep |
| **Use case** | Research, validation | Visualization, HRI |
| **Cost** | Free | Free (personal) |

### Decision Tree

```
Do you need accurate physics?
├─ Yes → Gazebo (SLAM, navigation, control)
└─ No  → Do you need beautiful graphics?
         ├─ Yes → Unity (VR, visualization)
         └─ No  → Gazebo (lighter-weight)

Is user interaction important?
├─ Yes → Unity (teleoperation, HRI)
└─ No  → Gazebo (autonomous algorithms)
```

---

## Part 2: URDF Import to Unity

### Unity Robotics Hub

**Prerequisites**:
- Unity 2020 LTS or newer
- URDF Importer plugin

**Installation**:
1. Download [Unity Robotics Hub](https://github.com/Unity-Robotics/URDF-Importer)
2. Import into Unity Assets/

**Import URDF**:
1. In Unity: `Assets > Import > URDF`
2. Select robot.urdf file
3. Importer creates GameObjects for each link/joint

---

## Part 3: Visualization in Unity

### Example Scene Setup

```csharp
using UnityEngine;
using RosSharp.Urdf;

public class RobotVisualizer : MonoBehaviour
{
    public UrdfRobot robot;

    void Start()
    {
        // Robot automatically loaded from URDF
        robot.SetJointRotation("shoulder_joint", 0.5f);  // radians
    }

    void Update()
    {
        // Update robot pose from ROS 2
        // Example: subscribe to joint_states topic
    }
}
```

### Material and Lighting

```csharp
void SetupLighting()
{
    // Create sun light for realistic shadows
    GameObject sun = new GameObject("Sun");
    Light sunLight = sun.AddComponent<Light>();
    sunLight.type = LightType.Directional;
    sunLight.intensity = 1.0f;
    sun.transform.rotation = Quaternion.Euler(45, 45, 0);
}
```

---

## Part 4: ROS 2 Integration in Unity

### ROS 2 Bridge for Unity

**Package**: `ros2-unity` (community project)

**Example: Subscribe to Joint States**

```csharp
using ROS2;
using sensor_msgs.msg;

public class JointStateListener : MonoBehaviour
{
    private ROS2Node ros2Node;
    private ISubscription<JointState> subscription;

    void Start()
    {
        ros2Node = GetComponent<ROS2Node>();
        subscription = ros2Node.CreateSubscription<JointState>(
            "/joint_states",
            OnJointStateReceived
        );
    }

    void OnJointStateReceived(JointState msg)
    {
        // Update robot visual based on joint states
        for (int i = 0; i < msg.name.Count; i++)
        {
            string jointName = msg.name[i];
            float position = (float)msg.position[i];
            // Apply to Unity joint (HingeJoint, ConfigurableJoint)
        }
    }
}
```

### Publish Commands to ROS 2

```csharp
using std_msgs.msg;

public class CommandPublisher : MonoBehaviour
{
    private IPublisher<Float32MultiArray> commandPublisher;

    void Start()
    {
        ROS2Node ros2Node = GetComponent<ROS2Node>();
        commandPublisher = ros2Node.CreatePublisher<Float32MultiArray>("/joint_commands");
    }

    void PublishCommand(float[] angles)
    {
        var msg = new Float32MultiArray();
        msg.data = angles;
        commandPublisher.Publish(msg);
    }
}
```

---

## Part 5: When to Use Each Tool

### Use Gazebo When:
- ✅ Developing control algorithms (SLAM, navigation)
- ✅ Testing collision and contact
- ✅ Need accurate physics for validation
- ✅ Headless simulation (no GPU needed)
- ✅ Research and academic work

### Use Unity When:
- ✅ Visualizing for humans (teleoperation)
- ✅ Virtual reality (VR) applications
- ✅ Human-robot interaction demos
- ✅ End-user facing simulation
- ✅ Graphics are primary goal

### Use Both:
- ✅ Gazebo for physics + control algorithm development
- ✅ Unity for visualization of same URDF
- ✅ Synchronize via ROS 2 topics
- ✅ Gazebo runs headless, Unity visualizes results

---

## Part 6: Hybrid Workflow

Run Gazebo for physics, visualize in Unity:

```bash
# Terminal 1: Gazebo (headless, physics accurate)
ros2 launch my_robot gazebo.launch.xml use_gui:=false

# Terminal 2: Unity (beautiful graphics)
# Open Unity scene with same URDF + ROS 2 bridge
# Subscribe to /joint_states from Gazebo
# Automatically renders synchronized animation
```

---

## Summary

**Gazebo**: Physics-accurate, ROS 2 native, research
**Unity**: Beautiful visuals, HRI, end-user facing

**Best practice**: Use Gazebo for algorithm development, Unity for visualization.

---

## Navigation

- **Previous**: [Chapter 3: Sensors](./03-sensors-in-gazebo.md)
- **Next**: [Chapter 5: Sim-to-Real](./05-sim-to-real-considerations.md)
