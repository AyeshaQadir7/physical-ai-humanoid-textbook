"use client";
import Link from "@docusaurus/Link";
import {
  MoveRight,
  Bot,
  Cpu,
  Zap,
  ChevronRight,
  ArrowRight,
} from "lucide-react";
// import { Button } from "@/components/ui/button";
// import { Badge } from "@/components/ui/badge";

export default function HeroSection() {
  return (
    <section className="relative min-h-screen flex flex-col items-center justify-center overflow-hidden px-4 md:px-12 bg-[#0a0a0a]">
      {/* Dynamic Background Elements */}
      <div className="absolute inset-0 z-0 pointer-events-none">
        <div className="absolute top-[-10%] left-[-10%] w-[40%] h-[40%] bg-primary/10 rounded-full blur-[120px]" />
        <div className="absolute bottom-[-10%] right-[-10%] w-[40%] h-[40%] bg-secondary/10 rounded-full blur-[120px]" />
        <div
          className="absolute inset-0 opacity-[0.03]"
          style={{
            backgroundImage: `radial-gradient(circle at 1px 1px, white 1px, transparent 0)`,
            backgroundSize: "40px 40px",
          }}
        />
      </div>

      {/* Main Content Area */}
      <div className="relative z-10 container max-w-7xl mx-auto flex flex-col lg:flex-row items-center gap-16 pt-12 pb-20">
        {/* Left Column: Text Content */}
        <div className="flex-1 text-center lg:text-left space-y-8">
          {/* badge */}
          <div className="inline-flex items-center justify-center">
            <span
              className="px-4 py-1.5 text-xs font-semibold uppercase tracking-wider rounded-full relative overflow-hidden"
              style={{
                backgroundColor: "rgba(34, 253, 255, 0.1)",
                color: "#22FDFF",
                border: "1px solid rgba(34, 253, 255, 0.2)",
              }}
            >
              <span className="relative z-10">
                <Zap size={14} className="mr-2 " />
                2026 Academic Edition
              </span>
              <span
                className="absolute inset-0 -translate-x-full animate-[shimmer_2s_infinite]"
                style={{
                  background:
                    "linear-gradient(90deg, transparent, rgba(34, 253, 255, 0.3), transparent)",
                }}
              />
            </span>
          </div>

          <h1 className="text-4xl md:text-6xl lg:text-7xl font-medium tracking-tight leading-[0.95] text-balance text-white">
            The Physics of{" "}
            <span className="text-primary italic text-[#22FDFF]">Embodied</span>{" "}
            Intelligence.
          </h1>

          <p
            className="text-lg md:text-xl max-w-2xl mx-auto lg:mx-0 leading-relaxed text-balance opacity-80"
            style={{ color: "#a0a0a0" }}
          >
            Master the frontier of Physical AI and Humanoid Robotics. Bridging
            the gap between digital neural networks and the kinetic reality of
            biological systems.
          </p>

          <div className="flex flex-col sm:flex-row items-center justify-center lg:justify-start gap-4 pt-4">
            {/* <Button
              size="lg"
              className="h-14 px-8 rounded-full bg-primary text-background hover:bg-primary/90 text-lg font-semibold group shadow-[0_0_20px_rgba(var(--primary),0.3)]"
            >
              Start Learning
              <MoveRight className="ml-2 group-hover:translate-x-1 transition-transform" />
            </Button> */}
            <Link href="/docs/course-intro">
              <button
                className="group relative border-none text-lg px-8 py-4 rounded-full cursor-pointer text-white font-semibold transition-all duration-300 overflow-hidden"
                style={{
                  // linear-gradient(135deg, #008080 0%, #22fdff 100%)
                  backgroundColor: "#008080",
                }}
              >
                <div
                  className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300 blur-xl"
                  style={{
                    //
                    background: "#00a0a0",
                  }}
                />
                <span className="relative z-10 flex items-center gap-2">
                  Start Learning
                  <ArrowRight strokeWidth={1.5} />
                </span>
              </button>
            </Link>
            {/* <Button
              variant="ghost"
              size="lg"
              className="h-14 px-8 rounded-full text-lg hover:bg-white/5"
            >
              Explore Modules
              <ChevronRight className="ml-1" size={18} />
            </Button> */}
            <Link href="/docs/module-1-ros2/module-1-intro">
              <button
                className="group relative border-2 text-lg px-8 py-4 rounded-full cursor-pointer font-semibold transition-all duration-300 overflow-hidden"
                style={{
                  backgroundColor: "transparent",
                  borderColor: "#ffffff",
                  color: "#ffffff",
                }}
              >
                <div
                  className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300 blur-xl"
                  style={{
                    background: "#242424ff",
                  }}
                />
                <span className="relative z-10 flex items-center gap-2">
                  Explore Modules
                </span>
              </button>
            </Link>
          </div>

          {/* Trust Indicators */}
          <div className="flex flex-wrap items-center justify-center lg:justify-start gap-8 opacity-60 grayscale hover:grayscale-0 transition-all duration-500">
            <div className="flex items-center gap-2">
              <Bot size={20} /> <span className="font-bold">NeuralCore</span>
            </div>
            <div className="flex items-center gap-2">
              <Cpu size={20} />{" "}
              <span className="font-bold">KineticDynamics</span>
            </div>
          </div>
        </div>

        {/* Right Column: Visual Component */}
        <div className="flex-1 relative w-full aspect-square max-w-xl group">
          {/* Main Hero Visual Card */}
          <div className="relative w-full h-full rounded-[2.5rem] overflow-hidden border border-white/10 bg-gradient-to-br from-white/15 to-transparent backdrop-blur-sm p-4 transition-transform duration-700 ease-out shadow-2xl">
            <div className="absolute inset-0 bg-primary/5 opacity-0 group-hover:opacity-100 transition-opacity duration-700" />

            <div className="relative h-full w-full rounded-[2rem] overflow-hidden">
              <img
                src="/img/roboimg.jpg"
                alt="Humanoid Robotics Concept"
                className="w-full h-full object-cover transition-transform duration-1000 ease-out"
              />

              {/* Floating Stat Overlay */}
              <div className="absolute bottom-6 left-6 right-6 p-6 bg-black/60 backdrop-blur-md border border-white/10 rounded-2xl flex items-center justify-between opacity-100 transition-all duration-500 delay-100">
                <div className="space-y-1">
                  <p
                    className="text-xs uppercase tracking-widest font-bold"
                    style={{ color: "#808080" }}
                  >
                    Latency
                  </p>
                  <p className="text-2xl font-mono  text-white">0.42ms</p>
                </div>
                <div className="h-10 w-px bg-white/10" />
                <div className="space-y-1">
                  <p
                    className="text-xs uppercase tracking-widest font-bold"
                    style={{ color: "#808080" }}
                  >
                    Accuracy
                  </p>
                  <p className="text-2xl font-mono  text-white">99.8%</p>
                </div>
              </div>
            </div>
          </div>

          {/* Decorative Accents */}
          <div className="absolute -top-6 -right-6 w-32 h-32 bg-secondary/20 rounded-full blur-3xl" />
          <div className="absolute -bottom-10 -left-10 w-40 h-40 bg-primary/20 rounded-full blur-3xl" />
        </div>
      </div>
    </section>
  );
}
