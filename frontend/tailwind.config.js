/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx,mdx}',
    './docs/**/*.{md,mdx}',
    './blog/**/*.{md,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        'void': '#0a0a0a',
        'dark-surface': '#1a1a1a',
        'cyan': '#22FDFF',
        'teal': '#008080',
      },
      fontSize: {
        'hero': 'clamp(2.5rem, 10vw, 7rem)',
      },
      letterSpacing: {
        'extreme': '0.15em',
      },
    },
  },
  plugins: [],
  corePlugins: {
    preflight: false,
  },
};
