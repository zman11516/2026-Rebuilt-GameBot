#!/usr/bin/env node
// Static server for the shot simulator web UI (no dependencies, read-only).
//
//   node tools/shotsim/serve.mjs [--port 5454]
//
// Serves tools/shotsim/ at / and the committed maps in src/main/deploy/shotmaps/ at /deploy/shotmaps/.

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const TOOL = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(TOOL, '..', '..');
const MOUNTS = [
  ['/deploy/shotmaps/', path.join(ROOT, 'src', 'main', 'deploy', 'shotmaps')],
  ['/', TOOL],
];
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.csv': 'text/csv; charset=utf-8',
  '.md': 'text/markdown; charset=utf-8',
};

const argv = process.argv.slice(2);
const portArg = argv.indexOf('--port');
const port = portArg >= 0 ? Number(argv[portArg + 1]) : 5454;

/** Map a URL path to a file under one of the mounts, or null if it escapes them. */
function resolve(urlPath) {
  const [prefix, dir] = MOUNTS.find(([p]) => urlPath.startsWith(p));
  const file = path.resolve(dir, '.' + path.posix.normalize('/' + urlPath.slice(prefix.length)));
  return file === dir || file.startsWith(dir + path.sep) ? file : null;
}

const server = http.createServer((req, res) => {
  const send = (code, body, type = 'text/plain; charset=utf-8') => {
    res.writeHead(code, { 'Content-Type': type, 'Cache-Control': 'no-store' });
    res.end(req.method === 'HEAD' ? undefined : body);
  };
  if (req.method !== 'GET' && req.method !== 'HEAD') return send(405, 'read-only\n');
  let urlPath;
  try {
    urlPath = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
  } catch {
    return send(400, 'bad path\n');
  }
  let file = resolve(urlPath);
  if (file && fs.existsSync(file) && fs.statSync(file).isDirectory()) file = path.join(file, 'index.html');
  const type = file && TYPES[path.extname(file)];
  if (!type || !fs.existsSync(file)) return send(404, 'not found\n');
  fs.readFile(file, (err, data) => (err ? send(500, String(err)) : send(200, data, type)));
});

server.listen(port, '127.0.0.1', () => {
  console.log(`shot simulator: http://localhost:${port}/`);
  console.log(`committed maps: http://localhost:${port}/deploy/shotmaps/hub.json`);
});
