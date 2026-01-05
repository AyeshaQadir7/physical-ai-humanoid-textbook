"use client";

import Link from "@docusaurus/Link";
import { ArrowRight, MessageCircleMore } from "lucide-react";
import React from "react";

/**
 * FinalCTA - Final call-to-action section at the end of the landing page
 * Features: Newsletter signup and action buttons with glow effects
 *
 * Design Specs:
 * - Background: #0a0a0a (Deep Void)
 * - Title: #ffffff (Pure White)
 * - Input fields: #1a1a1a (Dark Surface) with #008080 (Teal) borders
 * - Subscribe button: #22FDFF (Electric Cyan)
 * - Text: #F8F9FD (Light Gray)
 */

export default function FinalCTA(): React.ReactNode {
  const [email, setEmail] = React.useState("");
  const [name, setName] = React.useState("");

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // TODO: Connect to actual newsletter service
    console.log("Newsletter signup:", { name, email });
    setName("");
    setEmail("");
  };

  return (
    <section
      className="relative w-full px-4 py-20 md:py-32 overflow-hidden"
      style={{ backgroundColor: "#0a0a0a" }}
    >
      {/* Centered teal gradient blob */}
      <div
        className="absolute inset-0 overflow-hidden pointer-events-none"
        style={{
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
        }}
      >
        <div
          className="absolute"
          style={{
            width: "300px",
            height: "300px",
            background:
              "radial-gradient(circle at center, rgba(0, 128, 128, 0.3) 0%, rgba(34, 253, 255, 0.15) 40%, transparent 70%)",
            borderRadius: "50%",
            filter: "blur(60px)",
            zIndex: 0,
          }}
          aria-hidden="true"
        />
      </div>

      <div className="relative z-10 max-w-4xl mx-auto">
        <div className="flex flex-col justify-center items-center text-center mb-12 gap-4">
          <h3
            className="text-4xl md:text-5xl lg:text-6xl font-bold leading-tight text-balance"
            style={{ color: "#ffffff" }}
          >
            Ready to Build{" "}
            <span
              className="relative inline-block italic"
              style={{ color: "#22FDFF" }}
            >
              Intelligent Robots?
              <div
                className="absolute inset-0 overflow-hidden"
                style={{ pointerEvents: "none" }}
              ></div>
            </span>
          </h3>
          <p
            className="text-lg  max-w-2xl leading-relaxed"
            style={{ color: "#F8F9FD" }}
          >
            Join thousands of engineers mastering the future of physical AI.
          </p>
        </div>

        <div className="flex flex-wrap justify-center items-center gap-4 md:gap-6">
          <Link href="/docs/course-intro">
            <button
              className="group relative border-none text-lg px-8 py-4 rounded-full cursor-pointer text-white font-semibold transition-all duration-300 overflow-hidden"
              style={{
                backgroundColor: "#008080",
              }}
            >
              <div
                className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300 blur-xl"
                style={{
                  background: "#00a0a0",
                }}
              />
              <span className="relative z-10 flex items-center gap-2">
                Start Learning
                <ArrowRight strokeWidth={1.5} />
              </span>
            </button>
          </Link>

          <button
            className="group relative border-2 text-lg px-8 py-4 rounded-full cursor-pointer font-semibold transition-all duration-300 overflow-hidden"
            style={{
              backgroundColor: "transparent",
              borderColor: "#ffffff",
              color: "#ffffff",
            }}
          >
            <div
              className="absolute inset-0 opacity-0 group-hover:opacity-10 transition-opacity duration-300"
              style={{ backgroundColor: "#ffffff" }}
            />
            <span className="relative z-10 flex items-center gap-2">
              <MessageCircleMore strokeWidth={1.5} />
              Chatbot
            </span>
          </button>
        </div>
      </div>
    </section>
  );
}
