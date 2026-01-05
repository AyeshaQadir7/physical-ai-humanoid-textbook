import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "AI Systems in the Physical World. Embodied Intelligence.",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://physical-ai-robotics.vercel.app/",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "AyeshaQadir7", // Usually your GitHub org/user name.
  projectName: "physical-ai-robotics-textbook", // Usually your repo name.

  onBrokenLinks: "warn", // Changed from "throw" to allow incomplete content generation

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/AyeshaQadir7/physical-ai-robotics-textbook",
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/AyeshaQadir7/physical-ai-robotics-textbook/tree/main/frontend/",
          // Useful options to enforce blogging best practices
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Robotics",
      logo: {
        alt: "Physical AI & Robotics Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Textbook",
        },

        {
          href: "https://github.com/AyeshaQadir7/physical-ai-robotics-textbook",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Start Reading",
              to: "docs/course-intro",
            },
            {
              label: "Course Overview",
              to: "/docs/module-1-ros2/module-1-intro",
            },
            {
              label: "Prerequisites",
              to: "/docs/module-0-prerequisites/module-0-prerequisites-intro",
            },
            {
              label: "Hardware Setup",
              to: "/docs/hardware-setup/hw-minimum-requirements",
            },
          ],
        },
        {
          title: "Modules",
          items: [
            {
              label: "Moduel 1",
              href: "/docs/module-1-ros2/module-1-intro",
            },
            {
              label: "Moudle 2",
              href: "/docs/module-2-simulation/module-2-intro",
            },
            {
              label: "Module 3",
              href: "/docs/module-3-isaac/module-3-intro",
            },
            {
              label: "Module 4",
              href: "/docs/module-4-vla/module-4-intro",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/AyeshaQadir7/physical-ai-robotics-textbook",
            },
            {
              label: "Author",
              href: "https://github.com/AyeshaQadir7",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Robotics. Built with Docusaurus and Speckit Plus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
