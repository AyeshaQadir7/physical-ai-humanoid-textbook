import React from "react";

/**
 * ModuleCards - Course modules grid section
 * Features: Four module cards with clear visual hierarchy and progression
 *
 * Design Specs:
 * - Background: #1a1a1a (Dark Surface)
 * - Card background: #0a0a0a (Deep Void)
 * - Module numbers: #22FDFF (Electric Cyan)
 * - Titles: #ffffff (Pure White)
 * - Content: #F8F9FD (Light Gray)
 * - Accent line: #008080 (Teal)
 */

const modules = [
  {
    number: "01",
    title: "Learn Foundations",
    description: "Understand the fundamentals of Physical AI and ROS 2",
  },
  {
    number: "02",
    title: "Build Simulations",
    description: "Create digital twins and test behaviors safely",
  },
  {
    number: "03",
    title: "Integrate AI",
    description: "Add perception and intelligence to robots",
  },
  {
    number: "04",
    title: "Deploy & Control",
    description: "Build autonomous systems with natural interaction",
  },
];

export default function ModuleCards(): React.ReactNode {
  return (
    <section
      className="w-full px-4 py-20"
      style={{ backgroundColor: "#111111ff" }}
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
              <span className="relative z-10">Course Modules</span>
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
            Proven process for success
          </h2>
          <p
            className="text-base md:text-lg max-w-2xl mx-auto"
            style={{ color: "#a0a0a0" }}
          >
            We help you on every step of the journey
          </p>
        </div>

        {/* Modules - Horizontal Process Layout */}
        <div className="flex flex-col md:flex-row justify-center gap-12 md:gap-8">
          {modules.map((module, index) => (
            <div key={index} className="group flex-1 text-center">
              {/* Module Number - Large and prominent */}
              <div
                className="text-6xl md:text-7xl font-black mb-6"
                style={{ color: "#008080" }}
              >
                {module.number}
              </div>

              {/* Title */}
              <h3
                className="text-xl md:text-2xl font-bold text-white mb-4"
                style={{ color: "#ffffff" }}
              >
                {module.title}
              </h3>

              {/* Description */}
              <p
                className="text-sm md:text-base leading-relaxed"
                style={{ color: "#F8F9FD" }}
              >
                {module.description}
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
