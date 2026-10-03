/**
 * Render the two OffSeason-195 Markdown guides' Mermaid blocks to adjacent SVGs.
 * This is documentation tooling only; nothing here is part of the robot deploy tree.
 *
 * From repo root, install local tooling (or pass existing package roots):
 *   npm install --prefix artifacts/diagram-renderer --no-audit --no-fund mermaid@11.12.2 playwright@1.62.1
 *   node artifacts/diagram-renderer/node_modules/playwright/cli.js install chromium
 *   node tools/render_diagrams.mjs
 *
 * Options: --mermaid-root <package-dir>, --playwright-root <package-dir>,
 *          --browser-executable <file>, --check
 * --check parses/renders and verifies checked-in SVGs without overwriting them.
 * Preview PNGs and the geometry report always go to ignored artifacts/diagram-preview/.
 * A loopback-only HTTP server serves the local Mermaid package; no external page is loaded.
 */
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {fileURLToPath, pathToFileURL} from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const guideRoot = path.join(root, 'docs/offseason-195');
const previewRoot = path.join(root, 'artifacts/diagram-preview');
const localPackages = path.join(root, 'artifacts/diagram-renderer/node_modules');
const options = {mermaidRoot: path.join(localPackages, 'mermaid'),
  playwrightRoot: path.join(localPackages, 'playwright'), check: false};
const argumentsList = process.argv.slice(2);
while (argumentsList.length) {
  const argument = argumentsList.shift();
  if (argument === '--check') options.check = true;
  else {
    const key = {'--mermaid-root': 'mermaidRoot', '--playwright-root': 'playwrightRoot',
      '--browser-executable': 'browserExecutable'}[argument];
    assert(key && argumentsList.length, `Unknown or incomplete argument: ${argument}`);
    options[key] = path.resolve(argumentsList.shift());
  }
}

const guides = ['team-overview.md', 'programming-diagrams.md'];
const diagrams = [];
const names = new Set();
for (const guide of guides) {
  const markdown = await fs.readFile(path.join(guideRoot, guide), 'utf8');
  const images = [...markdown.matchAll(/!\[([^\]]+)\]\(diagrams\/([a-z0-9-]+)\.svg\)/g)];
  const blocks = [...markdown.matchAll(/```mermaid\r?\n([\s\S]*?)\r?\n```/g)];
  assert(images.length === blocks.length && blocks.length > 0, `${guide}: images and Mermaid must pair`);
  for (let index = 0; index < blocks.length; index++) {
    const [, alt, name] = images[index];
    assert(images[index].index < blocks[index].index, `${name}: image must precede its source`);
    assert(!names.has(name), `Duplicate diagram name: ${name}`);
    names.add(name);
    diagrams.push({guide, name, alt, source: blocks[index][1].replace(/\r\n/g, '\n')});
  }
}
await fs.mkdir(path.join(guideRoot, 'diagrams'), {recursive: true});
await fs.mkdir(previewRoot, {recursive: true});
const mermaidDist = path.join(options.mermaidRoot, 'dist');
const mermaidPackage = JSON.parse(await fs.readFile(path.join(options.mermaidRoot, 'package.json'), 'utf8'));
const {chromium} = await import(pathToFileURL(path.join(options.playwrightRoot, 'index.mjs')).href);
const server = http.createServer(async (request, response) => {
  try {
    const url = new URL(request.url, 'http://127.0.0.1');
    if (url.pathname === '/') {
      response.setHeader('Content-Type', 'text/html; charset=utf-8');
      response.end('<!doctype html><html><head><meta charset="utf-8">'
        + '<style>body{margin:24px;background:white;}#diagram{display:inline-block;}</style></head>'
        + '<body><div id="diagram"></div><script type="module">'
        + 'import mermaid from "/mermaid/mermaid.esm.min.mjs";'
        + 'window.mermaid = mermaid;window.mermaidReady = true;'
        + '</script></body></html>');
      return;
    }
    if (!url.pathname.startsWith('/mermaid/')) {
      response.writeHead(404); response.end(); return;
    }
    const asset = path.resolve(mermaidDist, decodeURIComponent(url.pathname.slice('/mermaid/'.length)));
    assert(asset.startsWith(mermaidDist + path.sep), 'Asset must remain in Mermaid package');
    response.setHeader('Content-Type', asset.endsWith('.json') ? 'application/json' : 'text/javascript');
    response.end(await fs.readFile(asset));
  } catch {
    response.writeHead(404); response.end();
  }
});
await new Promise((resolve, reject) => {
  server.once('error', reject);
  server.listen(0, '127.0.0.1', resolve);
});
let browser;
try {
  browser = await chromium.launch({headless: true,
    ...(options.browserExecutable ? {executablePath: options.browserExecutable} : {})});
  const page = await browser.newPage({viewport: {width: 1800, height: 1400}, deviceScaleFactor: 1});
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.waitForFunction(() => window.mermaidReady);
  const report = {mermaidVersion: mermaidPackage.version, diagrams: []};
  for (const diagram of diagrams) {
    const rendered = await page.evaluate(async ({source, name, alt}) => {
      window.mermaid.initialize({startOnLoad: false, securityLevel: 'strict',
        deterministicIds: true, deterministicIDSeed: name, theme: 'base',
        themeVariables: {fontFamily: 'Arial, sans-serif', fontSize: '17px',
          primaryColor: '#edf3fc', primaryTextColor: '#17243b', primaryBorderColor: '#647996',
          lineColor: '#52657e', secondaryColor: '#f3f5f8', tertiaryColor: '#ffffff'},
        flowchart: {htmlLabels: false, useMaxWidth: false, curve: 'linear', padding: 12,
          nodeSpacing: 24, rankSpacing: 30, wrappingWidth: 200},
        sequence: {useMaxWidth: false, wrap: true, width: 140, actorMargin: 30,
          messageMargin: 30, diagramMarginX: 20, diagramMarginY: 20}});
      await window.mermaid.parse(source);
      const result = await window.mermaid.render(name, source);
      document.querySelector('#diagram').innerHTML = result.svg;
      await document.fonts.ready;
      const svg = document.querySelector('#diagram svg');
      svg.setAttribute('role', 'img');
      svg.setAttribute('aria-label', alt);
      svg.style.backgroundColor = '#ffffff';
      const title = document.createElementNS('http://www.w3.org/2000/svg', 'title');
      title.textContent = alt;
      svg.prepend(title);
      const bounds = svg.getBoundingClientRect();
      const overflow = [...svg.querySelectorAll('text')].filter(element => {
        const box = element.getBoundingClientRect();
        return box.width && (box.left < bounds.left - 2 || box.right > bounds.right + 2
          || box.top < bounds.top - 2 || box.bottom > bounds.bottom + 2);
      }).map(element => element.textContent);
      return {svg: svg.outerHTML + '\n', width: Math.round(bounds.width),
        height: Math.round(bounds.height), textCount: svg.querySelectorAll('text').length, overflow};
    }, diagram);
    assert(rendered.textCount > 0 && !rendered.overflow.length,
      `${diagram.name}: missing or out-of-bounds text: ${rendered.overflow.join('; ')}`);
    const destination = path.join(guideRoot, 'diagrams', diagram.name + '.svg');
    if (options.check) {
      assert.equal((await fs.readFile(destination, 'utf8')).replace(/\r\n/g, '\n'), rendered.svg,
        `${diagram.name}: checked-in SVG differs; regenerate with the same Mermaid version`);
    } else await fs.writeFile(destination, rendered.svg);
    await page.locator('#diagram svg').screenshot({path: path.join(previewRoot, diagram.name + '.png')});
    const {svg, ...geometry} = rendered;
    report.diagrams.push({guide: diagram.guide, name: diagram.name, ...geometry});
    console.log(`${diagram.name}: ${geometry.width} x ${geometry.height}, ${geometry.textCount} labels`);
  }
  await fs.writeFile(path.join(previewRoot, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(`${options.check ? 'Verified' : 'Rendered'} ${diagrams.length} diagrams with Mermaid ${mermaidPackage.version}`);
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
