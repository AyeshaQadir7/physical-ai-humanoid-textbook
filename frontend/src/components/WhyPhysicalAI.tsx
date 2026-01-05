import type React from "react";

/**
 * WhyPhysicalAI - Section explaining why physical AI and embodied intelligence matters
 * Features: Dark surface background, centered content, key highlights with improved visual design
 *
 * Design Specs:
 * - Background: #1a1a1a (Dark Surface)
 * - Title: #ffffff (Pure White), bold
 * - Accent: #008080 (Teal) for visual interest
 * - Text: #F8F9FD (Light Gray)
 */

export default function WhyPhysicalAI(): React.ReactNode {
  const highlights = [
    {
      number: "1",
      title: "Humanoid robots match our physical world",
      description:
        "Unlike digital-only AI, humanoid robots are designed to interact with human environments naturally. They use the same tools, navigate the same spaces, and perform tasks that benefit from embodied understanding.",
    },
    {
      number: "2",
      title: "From digital AI to embodied intelligence",
      description:
        "Physical AI represents a fundamental shift: moving beyond pure computation to intelligence grounded in real-world sensing, actuation, and interaction. This is how intelligence actually works in nature—and now in machines.",
    },
    {
      number: "3",
      title: "Practical applications across industries",
      description:
        "Manufacturing, healthcare, logistics, and research—embodied AI systems solve real problems in the physical world that digital systems cannot.",
    },
    {
      number: "4",
      title: "The next frontier of computing",
      description:
        "Just as the internet was the frontier of the 2000s, physical AI is the frontier of this decade. Learning to build embodied intelligent systems is the most valuable skill in robotics.",
    },
  ];

  return (
    <section
      className="w-full px-4 py-20"
      style={{ backgroundColor: "#111111ff" }}
    >
      <div className="max-w-6xl mx-auto">
        {/* Section Title */}
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
              <span className="relative z-10">Why Physical AI Matters</span>
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
            Why Physical AI Matters
          </h2>
          <p
            className="text-base md:text-md max-w-2xl mx-auto"
            style={{ color: "#a0a0a0" }}
          >
            The future of AI is embodied robots that understand physical world
            as deeply as humans do.
          </p>
        </div>

        {/* Key Highlights Grid */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8 md:gap-10">
          {highlights.map((highlight) => (
            <div
              key={highlight.number}
              className="group relative p-8 rounded-2xl transition-all duration-300"
              style={{
                backgroundColor: "rgba(255, 255, 255, 0.03)",
                border: "1px solid rgba(255, 255, 255, 0.08)",
              }}
            >
              {/* Teal accent border on hover */}
              <div
                className="absolute inset-0 rounded-2xl opacity-0 group-hover:opacity-100 transition-opacity duration-300"
                style={{
                  border: "1px solid rgba(0, 128, 128, 0.3)",
                  pointerEvents: "none",
                }}
              />

              {/* Number Badge */}
              <div className="flex items-start gap-5 mb-4">
                <div
                  className="w-14 h-14 rounded-xl flex-shrink-0 flex items-center justify-center text-white font-bold text-xl shadow-lg"
                  style={{
                    backgroundColor: "#008080",
                    boxShadow: "0 4px 20px rgba(0, 128, 128, 0.25)",
                  }}
                >
                  {highlight.number}
                </div>
                <h3
                  className="text-xl md:text-2xl font-bold text-white leading-tight pt-2"
                  style={{ color: "#ffffff" }}
                >
                  {highlight.title}
                </h3>
              </div>

              {/* Description */}
              <p
                className="text-base md:text-lg leading-relaxed pl-[76px]"
                style={{ color: "#F8F9FD" }}
              >
                {highlight.description}
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
