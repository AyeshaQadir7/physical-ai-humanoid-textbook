```markdown
# landing-page.md – UI Design Reference Prompt for Docusaurus

Use this file as the single source of truth when redesigning the landing page of the “Physical AI & Humanoid Robotics” book built with Speckit Plus + Docusaurus.

## Design Principles

- Brutal minimalism meets cyber-tech aesthetic — think “cyberpunk research lab” but clean and readable
- Extreme contrast: deep black void backgrounds with glowing teal/cyan accents
- Almost no decorative elements — every visual decision must serve clarity and hierarchy
- Heavy use of vertical rhythm and generous whitespace
- Typography-first: large, bold headings with generous line-height
- Subtle micro-animations on hover/focus (scale, glow, border pulse) — never distracting
- All interactive elements must feel “tactile” and responsive
- Mobile-first but heroic on large screens (full-bleed hero, dramatic scaling)

## Color Palette (Exact Values – Do Not Deviate)

| Usage                | Color         | Hex                              |
| -------------------- | ------------- | -------------------------------- |
| Primary Background   | Deep Void     | #0a0a0a                          |
| Secondary Background | Dark Surface  | #1a1a1a                          |
| Primary Text         | Pure White    | #ffffff                          |
| Secondary Text       | Light Gray    | #F8F9FD                          |
| Main Accent          | Teal          | #008080                          |
| CTA / Highlight      | Electric Cyan | #22FDFF                          |
| Hover Glow           | Cyan glow     | 0 0 20px rgba(34, 253, 255, 0.4) |

## Typography

- Hero title: 4.5–7 rem (responsive), extreme letter-spacing (0.15em)

## Technical Requirements (Docusaurus v3 + Tailwind/MDX)

- All components must be built with Tailwind CSS classes only (no custom CSS unless absolutely necessary)
- Hero section must be full viewport height (`min-h-screen`)
- Smooth scroll enabled
- Prefers-reduced-motion respected
- Keep performance optimized - fast loading, minimal bloat
- Maintain existing color theme and fonts from the current book/documentation site
- Use design inspiration images provided as reference for modern aesthetics
- Take complete design inspiration - structure, layout, and design from the given images

- All images in `/design-reference/landing-page.md` (already provided):
  - `ui-ref-1.png` → Hero inspiration
  - `ui-ref-3.png` → What You Will Learn section
  - `ui-ref-4.png` → Module cards grid
  - `ui-ref-5.png` → Final CTA section

## Exact Page Structure (Follow Order Strictly)

### Section 1 – Hero Section (Full viewport)

Inspired by: ui-ref-1.png

- Background: #0a0a0a with subtle noise texture or very faint grid
- Centered content
- Title: **Physical AI & Humanoid Robotics** (huge, #ffffff)
- Subtitle: “AI Systems in the Physical World. Embodied Intelligence.” (#F8F9FD)
- One-liner quote in monospace italics:
  > “Bridging the gap between the digital brain and the physical body.”
- Two prominent CTA buttons side-by-side (mobile: stacked):
  - Primary: **Start Learning** → #22FDFF
  - Secondary: **Explore Book** → #008080

# **Section 2. Why Physical AI Matters**

The book begins with this theme, so it must be a prominent homepage section:
Use highlights like:

- Humanoid robots excel because they match our physical world.
- Represents shift from digital-only AI → embodied intelligence.
  Short paragraph explaining the transformation from digital AI to physical robots.

---

# **Section 3. What You Will Learn (Learning Outcomes)**

Taken directly from the textbook:

- Understand Physical AI & embodied intelligence
- Master ROS 2 for robot control
- Simulate robots with Gazebo & Unity
- Develop using NVIDIA Isaac Platform
- Design humanoids for natural interaction
- Build conversational robotics with GPT/LLMs
  Present these as icons or cards for visual impact.

---

# **Section 4. Course Modules (Module Cards Grid)**

This is a MUST because your book is structured around modules.

### Module 1: **The Robotic Nervous System (ROS 2)**

- Nodes, Topics, Services
- Python + rclpy
- URDF for humanoids

### Module 2: **The Digital Twin (Gazebo & Unity)**

- Physics simulation
- Sensor simulation
- Human–robot interaction in Unity

### Module 3: **The AI-Robot Brain (NVIDIA Isaac)**

- Isaac Sim
- Isaac ROS
- VSLAM
- Nav2 path planning

### Module 4: **Vision-Language-Action (VLA)**

- OpenAI Whisper
- Natural Language → ROS2 actions
- Final capstone: Autonomous humanoid
  This grid should visually communicate the flow.

---

# **Section 5. Call to Action**

End with:

- **Start Reading**
- **Open the Chatbot**

---

## Final Notes for Designer/Developer

- No rounded corners larger than 8px — keep everything sharp
- All CTAs must have a clear glow state on hover/focus
- Use only the exact colors above — no gradients unless they are teal → cyan
- Aim for less than 2 second load time — optimize all reference images
- The overall feeling when someone lands on the page should be:  
  “This is the most serious, cutting-edge robotics book on the planet.”

Implement this exactly. No creative liberties outside these constraints.
```

```

```
