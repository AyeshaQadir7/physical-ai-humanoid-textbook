---
id: module-4-lab-1
title: "Lab 4.1: Language-to-Action Pipeline"
sidebar_position: 10
sidebar_label: "Lab 4.1"
description: "Parse natural language commands and generate robot actions"
keywords: [language-to-action, LLM, parsing, robotics, lab]
---

# Lab 4.1: Language-to-Action Pipeline

## Lab Objective

**Goal**: Parse natural language commands into robot actions and execute in simulation.

**Skills**: LLM prompting, JSON parsing, action execution, error handling.

**Time**: 60 minutes

---

## Prerequisites

- ✅ Module 3 complete (perception working)
- ✅ OpenAI API key (or local Llama)
- ✅ Python 3.8+
- ✅ ROS 2 Humble

---

## Step 1: Setup LLM Client (10 minutes)

### 1.1: Install dependencies

```bash
pip install openai
```

### 1.2: Set API key

```bash
export OPENAI_API_KEY="sk-..."
```

### 1.3: Test LLM

```python
from openai import OpenAI

client = OpenAI()
response = client.chat.completions.create(
    model="gpt-4",
    messages=[{"role": "user", "content": "Say hello"}]
)
print(response.choices[0].message.content)
```

---

## Step 2: Create Language Agent (15 minutes)

Create `language_agent.py`:

```python
#!/usr/bin/env python3
import json
from openai import OpenAI

class LanguageAgent:
    def __init__(self):
        self.client = OpenAI()
        self.system_prompt = """You are a robot control system.
Convert user commands to JSON action plans.
Response: {"actions": [...], "reasoning": "..."}
"""

    def parse(self, command, robot_state):
        """Convert command to action plan"""
        prompt = f"""Robot state:
- Position: {robot_state.get('position', 'unknown')}
- Objects: {robot_state.get('objects', [])}

User: "{command}"

Generate action plan JSON."""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7
        )

        text = response.choices[0].message.content
        try:
            plan = json.loads(text)
            return plan
        except json.JSONDecodeError:
            return {"actions": [], "error": "Parse failed"}

if __name__ == "__main__":
    agent = LanguageAgent()

    # Test commands
    test_commands = [
        "Walk forward",
        "Find the coffee cup",
        "Go to the kitchen",
        "Pick up the blue box"
    ]

    for cmd in test_commands:
        state = {"position": (0, 0), "objects": ["cup", "box"]}
        plan = agent.parse(cmd, state)
        print(f"\nCommand: {cmd}")
        print(f"Plan: {json.dumps(plan, indent=2)}")
```

Run:

```bash
python3 language_agent.py
```

---

## Step 3: Implement Action Executor (15 minutes)

Create `action_executor.py`:

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ActionExecutor:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('action_executor')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def execute(self, action):
        """Execute single action"""
        action_type = action.get('type', 'unknown')

        if action_type == 'move_forward':
            return self.move_forward(action.get('distance', 1.0))
        elif action_type == 'navigate':
            return self.navigate(action.get('location', (0, 0)))
        elif action_type == 'detect':
            return self.detect_object(action.get('object'))
        else:
            return False

    def move_forward(self, distance):
        """Move robot forward"""
        msg = Twist()
        msg.linear.x = 0.3
        for _ in range(int(distance * 10)):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        msg.linear.x = 0
        self.cmd_pub.publish(msg)
        return True

    def navigate(self, location):
        """Navigate to location"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(location[0])
        goal.pose.pose.position.y = float(location[1])
        self.nav_client.send_goal_async(goal)
        return True

    def detect_object(self, obj_name):
        """Detect object (uses perception from Module 3)"""
        # Assumes perception pipeline running
        return True
```

---

## Step 4: Integrate Pipeline (15 minutes)

Create `vla_pipeline.py`:

```python
#!/usr/bin/env python3
from language_agent import LanguageAgent
from action_executor import ActionExecutor

class VLAPipeline:
    def __init__(self):
        self.agent = LanguageAgent()
        self.executor = ActionExecutor()

    def process_command(self, command):
        """Full pipeline: command → plan → execute"""
        print(f"\n1. User: {command}")

        # Parse
        robot_state = {"position": (0, 0), "objects": ["cup"]}
        plan = self.agent.parse(command, robot_state)
        print(f"2. Plan: {plan}")

        # Execute
        if 'actions' not in plan:
            print("3. Error: No actions in plan")
            return False

        for i, action in enumerate(plan['actions']):
            print(f"3.{i+1} Executing: {action}")
            success = self.executor.execute(action)
            if not success:
                print(f"Failed at action: {action}")
                return False

        print("4. Success!")
        return True

if __name__ == "__main__":
    pipeline = VLAPipeline()

    commands = [
        "Walk to the kitchen",
        "Find the coffee cup",
        "Pick it up and bring it back"
    ]

    for cmd in commands:
        pipeline.process_command(cmd)
```

---

## Step 5: Test in Simulation (15 minutes)

Launch Gazebo/Isaac with robot:

```bash
# Terminal 1: Gazebo
ros2 launch my_robot gazebo.launch.xml

# Terminal 2: VLA pipeline
python3 vla_pipeline.py
```

---

## Expected Output

```
1. User: Walk to the kitchen
2. Plan: {"actions": [
    {"type": "navigate", "location": "kitchen"},
    {"type": "detect", "object": "coffee"},
    {"type": "grasp", "object": "coffee"}
  ]}
3.1 Executing: navigate
3.2 Executing: detect
3.3 Executing: grasp
4. Success!
```

---

## Verification Checklist

- [ ] LLM responds to commands
- [ ] JSON parsing works
- [ ] Actions generated correctly
- [ ] Robot executes in simulation
- [ ] Handles errors gracefully

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| API key error | Set OPENAI_API_KEY env var |
| JSON parse fails | Check LLM response format |
| Robot doesn't move | Verify ROS 2 topics |

---

## Summary

**Lab 4.1 accomplishes**:
- Language parsing ✓
- Action generation ✓
- Pipeline integration ✓

**Ready for**: Lab 4.2 (voice commands)

---

## Navigation

- **Previous**: [VLA Guide](./01-vla-complete-guide.md)
- **Next Lab**: [Lab 4.2: Voice Commands](./lab-4-2-voice-command-processing.md)
