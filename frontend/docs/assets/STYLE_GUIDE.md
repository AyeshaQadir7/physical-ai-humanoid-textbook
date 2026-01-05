# Physical AI & Humanoid Robotics Textbook — Style Guide

**Version**: 1.0
**Last Updated**: 2025-12-10

---

## Overview

This guide ensures consistent formatting, tone, and structure across all chapters of the Physical AI & Humanoid Robotics Textbook. All authors must follow these standards before submission.

---

## Markdown Standards

### Frontmatter (Required)

Every chapter **must** start with YAML frontmatter:

```yaml
---
id: module-X-chapter-Y
title: "Descriptive Chapter Title"
sidebar_position: N
sidebar_label: "Short Title (5-7 words)"
description: "One-sentence summary of chapter content (under 120 chars)"
keywords: [keyword1, keyword2, keyword3, keyword4]
---
```

**Rules**:
- `id`: Must be unique across all docs; use hyphens (no underscores)
- `sidebar_position`: Numeric; used for auto-sorting in navigation
- `sidebar_label`: Keep under 20 characters for readability
- `description`: Exactly one sentence; appears in search results
- `keywords`: 3-5 terms for search optimization

### Headings

Use H2 (`##`) as your top-level heading within a chapter (H1 is reserved for Docusaurus page title).

```markdown
## Main Section
### Subsection
#### Detailed Topic
```

**Rules**:
- Use descriptive heading text
- Avoid ALL CAPS headings
- Maximum nesting: 4 levels (#### is deepest)

### Lists

**Unordered lists**:
```markdown
- Item 1
- Item 2
  - Nested item 2a
  - Nested item 2b
- Item 3
```

**Ordered lists**:
```markdown
1. First step
2. Second step
3. Third step
```

**Rules**:
- Use consistent indentation (2 spaces per level)
- Capitalize first word after bullet/number
- Use periods for complete sentences; omit for fragments

### Emphasis & Formatting

```markdown
**Bold text** — for emphasis, key terms, class names
*Italic text* — for variables, file names, concepts
`Code snippet` — for inline code, commands, identifiers
***Bold and italic*** — rarely used; avoid if possible
```

**Rules**:
- Use bold for **ROS 2**, **Gazebo**, **Isaac Sim**, **VLA** (proper names)
- Use code backticks for `sensor_msgs`, `node.create_publisher()`, `/cmd_vel`
- Do not use all caps for emphasis (use bold instead)

### Links

**Internal links** (relative):
```markdown
[Learning Outcomes](../module-0-foundations/intro.md)
[See Glossary](#glossary-entry)
```

**External links** (absolute):
```markdown
[ROS 2 Documentation](https://docs.ros.org)
[SLAM Algorithms](https://scholar.google.com/...)
```

**Rules**:
- Internal links must use relative paths (no absolute URLs)
- Always provide descriptive link text (never "click here")
- Test all links before submission

### Code Blocks

Inline code (short):
```markdown
Use `rclpy.create_node()` to initialize a ROS 2 node.
```

Code block (language-tagged):
````markdown
```python
import rclpy
from rclpy.node import Node

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
```
````

**Rules**:
- Always specify language (`python`, `bash`, `yaml`, `cpp`, `xml`, etc.)
- Include comments explaining non-obvious code
- Keep code blocks under 30 lines (break into multiple if needed)
- Provide expected output in the text following the block

### Admonitions (Note, Tip, Warning, Danger)

```markdown
:::note
This is helpful context but not critical to understand the concept.
:::

:::tip
Pro tip: Here's a shortcut or best practice.
:::

:::warning
Important: Students should pay attention to this potential pitfall.
:::

:::danger
Critical: This could break your system or create safety issues.
:::
```

**Rules**:
- Use `:::note` for additional context
- Use `:::tip` for optimization or advanced techniques
- Use `:::warning` for common mistakes or troubleshooting
- Use `:::danger` for safety-critical information
- Each admonition should be a single paragraph (not multi-paragraph)

---

## Images & Diagrams

### File Naming

```
docs/assets/diagrams/ros2-graph-architecture.txt  (ASCII diagrams)
docs/assets/screenshots/gazebo-world-setup.png    (PNG screenshots)
docs/assets/code-snippets/talker-example.py       (Code files)
```

### Markdown Image Syntax

```markdown
![Alt text describing the image](../assets/diagrams/ros2-graph.txt)
```

**Rules**:
- All images must have descriptive alt-text (for accessibility)
- Alt-text should be 5-10 words
- Images should be < 500KB (resize if needed)
- Use relative paths (no absolute URLs to local images)

### ASCII Diagrams

For architecture, workflows, and data flow diagrams, use ASCII art:

```
    ┌─────────────┐
    │  Publisher  │
    └──────┬──────┘
           │ publishes /sensor_data
           ▼
    ┌─────────────┐
    │     ROS     │
    │     DDS     │
    └──────┬──────┘
           │ subscribes
           ▼
    ┌─────────────┐
    │ Subscriber  │
    └─────────────┘
```

---

## Chapter Structure Template

Every chapter should follow this structure:

```markdown
---
id: module-X-chapter-Y
title: "Full Title"
sidebar_position: N
sidebar_label: "Short"
description: "One sentence."
keywords: [key1, key2]
---

## Introduction

[Hook + context; why this matters; connection to capstone]

## Learning Outcomes

- Outcome 1
- Outcome 2
- Outcome 3

## Core Concepts

### Concept 1: [Name]

[Theory, explanation, key principles]

#### Sub-concept 1.1
[Details]

### Concept 2: [Name]

[Theory, explanation]

## Examples

### Example 1: [Description]

```python
# Code
```

### Example 2: [Description]

```bash
# Command
```

## Hands-On Lab: [Lab Title]

:::note
**Time Required**: X minutes
**Prerequisites**: Topic A, Topic B, installed software Z
:::

### Lab Overview

[What students will do; what they'll learn]

### Setup

[Step-by-step instructions to prepare environment]

### Step-by-Step Tasks

1. Task 1: [Description]
   ```bash
   command here
   ```
2. Task 2: [Description]
3. Task 3: [Description]

### Expected Output

```
Output expected here
```

### Verification Checklist

- [ ] Verify output matches expected
- [ ] Check for error messages
- [ ] Confirm topic is publishing

### Troubleshooting

**Issue**: Error message X
**Solution**: Do this...

**Issue**: Output doesn't match
**Solution**: Check that...

### Extension Challenge

Try this advanced variation...

## Summary & Next Steps

[Key takeaways; bridge to next chapter; glossary links]

## Further Reading

- [Resource 1](link)
- [Resource 2](link)

## Code Repository

[GitHub links to example implementations]
```

---

## Terminology & Glossary

### Terminology Standards

1. **Consistent Naming**:
   - **ROS 2** (not "ROS2" or "ROS 2.0")
   - **Gazebo** (not "gazebo" or "Gazebo Classic")
   - **URDF** (Unified Robot Description Format — spell out on first use)
   - **Vision-Language-Action** (VLA on subsequent mentions)
   - **SLAM** (Simultaneous Localization and Mapping — spell out on first use)

2. **Glossary Cross-Reference**:
   - On first mention of a new term, check `docs/glossary.md`
   - If defined in glossary, use inline link: `[term](../glossary.md#term)`
   - If not yet in glossary, add it immediately

3. **Code Terminology**:
   - Preserve exact casing for class names, function names, package names
   - Example: `rclpy`, `Publisher`, `sensor_msgs/Image`, `/cmd_vel` (topic names with `/`)

---

## Tone & Voice

### Principles

1. **Clear & Accessible**: Assume reader is learning this topic for the first time
2. **Encouraging**: Acknowledge difficulty; celebrate progress
3. **Precise**: Technical accuracy over simplicity (but don't sacrifice clarity)
4. **Conversational**: Use "you" and "we" sparingly; avoid overly formal language

### Examples

**Good tone**:
> "ROS 2 uses a publish-subscribe pattern, where one node publishes data (like sensor readings) and other nodes subscribe to receive that data. This decouples the producer from the consumer, making systems easier to extend."

**Avoid**:
> "The publish-subscribe pattern, which ROS 2 leverages, is a messaging paradigm wherein entities denominated as publishers disseminate data to interested subscribers in a topic-based architecture."

---

## Code Quality Standards

### Python Code

```python
# Good: Clear comments, follows PEP 8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """Publishes incrementing counter to /topic."""

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        """Publish next message."""
        msg = String()
        msg.data = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1

if __name__ == '__main__':
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
```

### Bash/Shell Code

```bash
# Always include comments for non-obvious commands
# Build ROS 2 package
colcon build --packages-select my_package

# Run the node
ros2 run my_package my_node
```

### YAML/XML Code

```yaml
# Clear, minimal example
name: my_package
version: 1.0.0
description: "Brief description"

dependencies:
  - rclpy
  - sensor_msgs
```

---

## Accessibility Standards

1. **Alt-Text for All Images**: Every image must have descriptive alt-text
2. **Color Not Only Signal**: Don't use color alone to convey information
3. **Code Blocks with Language Tags**: Helps screen readers identify code
4. **Heading Hierarchy**: Use correct heading levels (don't skip H3 to H5)
5. **Link Text**: Never use "click here"; use descriptive text

---

## Submission Checklist

Before submitting a chapter, ensure:

- [ ] Frontmatter is complete and unique `id`
- [ ] All headings use H2-H4 (no H1, no H5+)
- [ ] All code blocks have language tags
- [ ] All images have alt-text and descriptive filenames
- [ ] All links (internal and external) are tested and work
- [ ] No admonitions are longer than one paragraph
- [ ] Terminology consistent with glossary
- [ ] Lab section includes Prerequisites, Setup, Steps, Output, Checklist, Troubleshooting, Extension
- [ ] Chapter connects explicitly to capstone project
- [ ] No TODO, FIXME, or placeholder text remains
- [ ] Grammar and spelling checked
- [ ] Docusaurus build succeeds (`npm run build`) with no warnings

---

## Questions?

For questions about style, submit an issue to the textbook repository or contact the lead author.
