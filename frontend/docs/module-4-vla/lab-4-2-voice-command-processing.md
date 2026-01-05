---
id: module-4-lab-2
title: "Lab 4.2: Voice Command Processing"
sidebar_position: 11
sidebar_label: "Lab 4.2"
description: "Process voice input with Whisper and execute robot actions"
keywords: [Whisper, voice commands, ASR, speech recognition, lab]
---

# Lab 4.2: Voice Command Processing

## Lab Objective

**Goal**: Record voice, transcribe with Whisper, and execute resulting actions.

**Skills**: Audio processing, Whisper integration, real-time voice control.

**Time**: 45 minutes

---

## Prerequisites

- ✅ Lab 4.1 complete (language-to-action working)
- ✅ Microphone available
- ✅ `pip install openai-whisper pyaudio`

---

## Step 1: Install Whisper (5 minutes)

```bash
pip install openai-whisper pyaudio numpy
whisper --version
```

---

## Step 2: Create Voice Input Node (15 minutes)

Create `voice_input_node.py`:

```python
#!/usr/bin/env python3
import rclpy
import whisper
import pyaudio
import numpy as np
from std_msgs.msg import String

class VoiceInputNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('voice_input')
        self.pub = self.create_publisher(String, '/voice_command', 10)
        self.model = whisper.load_model("base")
        self.get_logger().info("Voice node ready. Listening...")

    def listen_and_transcribe(self, duration=3):
        """Record audio and transcribe"""
        # Record
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32, channels=1,
                       rate=16000, input=True, frames_per_buffer=1024)
        self.get_logger().info("Recording...")
        frames = []
        for _ in range(int(16000/1024 * duration)):
            data = stream.read(1024)
            frames.append(np.frombuffer(data, np.float32))
        stream.stop_stream()
        p.terminate()

        # Concatenate audio
        audio = np.concatenate(frames)

        # Transcribe
        result = self.model.transcribe(audio, language="en")
        text = result["text"].strip()
        return text

    def run(self):
        """Continuously listen and publish"""
        while rclpy.ok():
            text = self.listen_and_transcribe(duration=2)
            if text:
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"Heard: {text}")

if __name__ == '__main__':
    rclpy.init()
    node = VoiceInputNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
```

---

## Step 3: Connect to VLA Pipeline (15 minutes)

Create `voice_vla_integration.py`:

```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from language_agent import LanguageAgent
from action_executor import ActionExecutor

class VoiceVLAIntegration(rclpy.node.Node):
    def __init__(self):
        super().__init__('voice_vla')
        self.sub = self.create_subscription(String, '/voice_command', self.voice_callback, 10)
        self.agent = LanguageAgent()
        self.executor = ActionExecutor()
        self.get_logger().info("VLA system ready")

    def voice_callback(self, msg):
        """Process voice command"""
        command = msg.data
        self.get_logger().info(f"Voice command: {command}")

        # Parse
        state = {"position": (0, 0), "objects": []}
        plan = self.agent.parse(command, state)
        self.get_logger().info(f"Plan: {plan}")

        # Execute
        if 'actions' in plan:
            for action in plan['actions']:
                success = self.executor.execute(action)
                if not success:
                    self.get_logger().error(f"Failed: {action}")
                    break

        self.get_logger().info("Command complete")

if __name__ == '__main__':
    rclpy.init()
    node = VoiceVLAIntegration()
    rclpy.spin(node)
```

---

## Step 4: Test Voice Control (10 minutes)

Terminal 1: Launch robot

```bash
ros2 launch my_robot gazebo.launch.xml
```

Terminal 2: Start voice node

```bash
python3 voice_input_node.py
```

Terminal 3: Start VLA integration

```bash
python3 voice_vla_integration.py
```

Now speak commands:
- "Walk forward"
- "Find the cup"
- "Go home"

---

## Expected Output

```
[INFO] Recording...
[INFO] Heard: walk forward
[INFO] Plan: {"actions": [{"type": "move_forward", "distance": 1.0}]}
[INFO] Command complete
```

---

## Verification Checklist

- [ ] Whisper transcribes speech accurately
- [ ] LLM parses voice-to-action
- [ ] Robot executes voice commands
- [ ] Low latency (under 5s per command)

---

## Summary

**Lab 4.2 accomplishes**:
- Voice input working ✓
- Whisper integration ✓
- End-to-end voice control ✓

**Ready for**: Lab 4.3 (capstone integration)

---

## Navigation

- **Previous Lab**: [Lab 4.1: Language-to-Action](./lab-4-1-language-to-action-pipeline.md)
- **Next Lab**: [Lab 4.3: Capstone](./lab-4-3-capstone-project-autonomous-humanoid.md)
