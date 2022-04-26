module.exports = {
  content: [
    "./pages/**/*.{js,ts,jsx,tsx}",
    "./components/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      backgroundImage: {
        //'hero-pattern': "url('/public/images/hero-pattern.svg')",
        'footer-texture': "url('/public/images/Screen Shot 2022-04-05 at 10.51.47 AM.png')",
      }
    },
  },
  plugins: [],
}