"use client";

import {
  Bot,
  BotMessageSquare,
  BrainCircuit,
  ChartNoAxesColumn,
  Drone,
  Globe,
  Speech,
  Wrench,
  Zap,
} from "lucide-react";
import type React from "react";

/**
 * WhatYouLearn - Learning outcomes section with icon/card-based list
 * Features: Dark void background, card-based grid, icon representations
 *
 * Design Specs:
 * - Background: #0a0a0a (Deep Void)
 * - Cards: #1a1a1a (Dark Surface) with teal accents
 * - Title: #ffffff (Pure White)
 * - Text: #F8F9FD (Light Gray)
 * - Icons: #22FDFF (Electric Cyan)
 */

const learningOutcomes = [
  {
    icon: <BrainCircuit strokeWidth={1.25} />,
    title: "Embodied Intelligence",
    description: "Understand AI systems in the physical world",
  },
  {
    icon: <Bot strokeWidth={1.25} />,
    title: "Robot Control",
    description: "Master ROS 2 fundamentals",
  },
  {
    icon: <Globe strokeWidth={1.25} />,
    title: "Digital Twins",
    description: "Simulate with Gazebo & Unity",
  },
  {
    icon: <Zap strokeWidth={1.25} />,
    title: "NVIDIA Isaac",
    description: "Cut-edge robotics development",
  },
  {
    icon: <Drone strokeWidth={1} />,
    title: "Humanoid Design",
    description: "Natural robot interaction",
  },
  {
    icon: <BotMessageSquare strokeWidth={1} />,
    title: "Conversational AI",
    description: "Build language-based robots",
  },
  {
    icon: <Wrench strokeWidth={1} />,
    title: "Hardware Integration",
    description: "Motors, sensors, and actuators",
  },
  {
    icon: <ChartNoAxesColumn strokeWidth={1} />,
    title: "Advanced Analytics",
    description: "Monitor and optimize robot behavior",
  },
];

export default function WhatYouLearn(): React.ReactNode {
  return (
    <section
      className="w-full px-4 py-20"
      style={{ backgroundColor: "#0a0a0a" }}
    >
      <div className="max-w-7xl mx-auto">
        {/* Section Header */}
        <div className="mb-16 text-center">
          <div className="inline-flex items-center justify-center mb-6">
            <span
              className="px-4 py-1.5 text-xs font-semibold uppercase tracking-wider rounded-full relative overflow-hidden"
              style={{
                backgroundColor: "rgba(34, 253, 255, 0.1)",
                color: "#22FDFF",
                border: "1px solid rgba(34, 253, 255, 0.2)",
              }}
            >
              <span className="relative z-10">Learning Outcomes</span>
              <span
                className="absolute inset-0 -translate-x-full animate-[shimmer_2s_infinite]"
                style={{
                  background:
                    "linear-gradient(90deg, transparent, rgba(34, 253, 255, 0.3), transparent)",
                }}
              />
            </span>
          </div>
          <h2
            className="text-3xl md:text-5xl font-bold text-white mb-6 leading-tight"
            style={{ color: "#ffffff" }}
          >
            What You Will Learn
          </h2>
          <p
            className="text-base md:text-lg max-w-2xl mx-auto"
            style={{ color: "#a0a0a0" }}
          >
            A comprehensive curriculum covering every aspect of physical AI and
            robotics
          </p>
        </div>

        {/* Learning Outcomes Grid - 4 column layout matching reference */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
          {learningOutcomes.map((outcome, index) => (
            <div
              key={index}
              className="group p-4 rounded-2xl transition-all duration-300 hover:-translate-y-1"
              style={{
                backgroundColor: "#1a1a1a",
                border: "1px solid rgba(255, 255, 255, 0.05)",
              }}
              onMouseEnter={(e) => {
                const el = e.currentTarget;
                el.style.borderColor = "rgba(34, 253, 255, 0.3)";
                el.style.backgroundColor = "#1f1f1f";
              }}
              onMouseLeave={(e) => {
                const el = e.currentTarget;
                el.style.borderColor = "rgba(255, 255, 255, 0.05)";
                el.style.backgroundColor = "#1a1a1a";
              }}
            >
              {/* Icon Container */}
              <div
                className="w-12 h-12 rounded-full flex items-center justify-center mb-6 transition-all duration-300 group-hover:scale-110"
                style={{
                  backgroundColor: "rgba(34, 253, 255, 0.1)",
                  border: "1px solid rgba(34, 253, 255, 0.2)",
                }}
              >
                <span
                  className="text-2xl flex items-center"
                  style={{ color: "#ffffff" }}
                >
                  {outcome.icon}
                </span>
              </div>

              {/* Title */}
              <h3 className="text-lg font-bold text-white mb-2 text-left">
                {outcome.title}
              </h3>

              {/* Description */}
              <p
                className="text-sm leading-relaxed text-left"
                style={{ color: "#a0a0a0" }}
              >
                {outcome.description}
              </p>
            </div>
          ))}
        </div>
      </div>

      <style>{`
        @keyframes shimmer {
          0% {
            transform: translateX(-100%);
          }
          100% {
            transform: translateX(200%);
          }
        }
      `}</style>
    </section>
  );
}
