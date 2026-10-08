import { defineConfig } from 'vite';

export default defineConfig({
  server: { host: '127.0.0.1', port: Number(process.env.PORT) || 5173, open: false },
  base: './',
  build: { chunkSizeWarningLimit: 1500 }, // three.js 单包约 700 kB，属正常
});
