// Build the site's pages from the repository's Markdown: the Markdown in the repository stays the
// one source (GitHub, the PDF guides); this copies the pages listed in site.config.mjs into
// content/ and adapts them for VitePress:
//   - links between listed pages point to their site pages; other repository files to GitHub,
//     images are copied next to the pages, listed downloads (PDFs) go to /files/;
//   - Python-Markdown admonitions (!!! note "Title") become VitePress containers, attribute
//     lists ({: .x }) become markdown-it attributes ({.x});
//   - {{ and stray <placeholders> outside code are escaped (Vue would compile them);
//   - a guide with a version in its front matter gets a title block with its PDF.
// Usage: node scripts/sync.mjs (npm run build does it first).

import { execSync } from 'node:child_process'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

import site from '../site.config.mjs'

const SITE = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const ROOT = path.resolve(SITE, site.root ?? '../..')
const OUT = path.join(SITE, 'content')
const posix = path.posix
const IMAGE = /\.(png|jpe?g|gif|svg|webp)$/i
const HTML_TAGS = new Set(
  ('a abbr b blockquote br code dd del details div dl dt em figcaption figure h1 h2 h3 h4 h5 h6 ' +
    'hr i img kbd li ol p picture pre s samp small source span strong sub summary sup table ' +
    'tbody td tfoot th thead tr u ul video wbr center').split(' ')
)

export function pageList(config = site) {
  return config.sections.flatMap((section) => section.items.map((item) => ({ ...item, section })))
}

// ---------------------------------------------------------------- front matter

export function splitFrontMatter(text) {
  if (!text.startsWith('---\n')) return { yaml: '', body: text }
  const end = text.indexOf('\n---\n', 4)
  if (end < 0) return { yaml: '', body: text }
  return { yaml: text.slice(4, end), body: text.slice(end + 5) }
}

// Top-level scalars of a YAML front matter (plain, quoted or folded with >-), enough for a title.
export function scalars(yaml) {
  const out = {}
  const lines = yaml.split('\n')
  for (let i = 0; i < lines.length; i++) {
    const m = /^([A-Za-z_][\w-]*):\s*(.*)$/.exec(lines[i])
    if (!m) continue
    let value = m[2].trim()
    if (/^[>|]-?$/.test(value)) {
      const parts = []
      while (i + 1 < lines.length && /^\s+\S/.test(lines[i + 1])) parts.push(lines[++i].trim())
      value = parts.join(' ')
    } else if (/^["'].*["']$/.test(value)) {
      value = value.slice(1, -1)
    }
    if (value && !value.startsWith('[') && !value.startsWith('-')) out[m[1]] = value
  }
  return out
}

// ---------------------------------------------------------------- Markdown transforms

const ADMONITION_TYPES = { note: 'info', info: 'info', tip: 'tip', hint: 'tip', important: 'warning',
  warning: 'warning', caution: 'warning', attention: 'warning', danger: 'danger', error: 'danger' }

// Python-Markdown `!!! type "Title"` with a 4-space indented body -> `::: type Title ... :::`.
export function admonitions(lines) {
  const out = []
  for (let i = 0; i < lines.length; i++) {
    const m = /^!!!\s+(\w+)(?:\s+"([^"]*)")?\s*$/.exec(lines[i])
    if (!m) {
      out.push(lines[i])
      continue
    }
    const kind = ADMONITION_TYPES[m[1].toLowerCase()] ?? 'info'
    out.push(`::: ${kind}${m[2] ? ' ' + m[2] : ''}`)
    while (i + 1 < lines.length && (/^ {4}/.test(lines[i + 1]) || (lines[i + 1] === '' &&
      /^ {4}/.test(lines[i + 2] ?? '')))) {
      out.push(lines[++i].replace(/^ {4}/, ''))
    }
    out.push(':::')
  }
  return out
}

// Run fn on the parts of a line outside inline code spans.
function outsideInlineCode(line, fn) {
  return line
    .split(/(`+[^`]*`+)/)
    .map((part, i) => (i % 2 === 1 ? part : fn(part)))
    .join('')
}

export function escapeForVue(text) {
  return text
    .replace(/\{\{/g, '&#123;&#123;')
    .replace(/<\/?([A-Za-z_][\w.-]*)([^>]*)>/g, (all, name, rest) =>
      HTML_TAGS.has(name.toLowerCase()) || rest.startsWith(':') // an autolink <https://...>
        ? all
        : all.replace(/</g, '&lt;').replace(/>/g, '&gt;')
    )
}

// ---------------------------------------------------------------- links

function relativeLink(fromDest, toDest) {
  let rel = posix.relative(posix.dirname(fromDest), toDest)
  if (!rel.startsWith('.')) rel = './' + rel
  return rel
}

export function makeLinker({ pages, files, repoUrl, branch, exists, isDir, copyAsset }) {
  const bySource = new Map(pages.map((p) => [p.src, p.dest]))
  const byFile = new Map(files.map((f) => [f.src, f.as]))
  return function link(target, srcRel, destRel) {
    if (/^[a-z][a-z0-9+.-]*:/i.test(target) || target.startsWith('//') || target.startsWith('#')) {
      return target
    }
    const hashAt = target.indexOf('#')
    const hash = hashAt >= 0 ? target.slice(hashAt) : ''
    const bare = (hashAt >= 0 ? target.slice(0, hashAt) : target).split('?')[0]
    if (!bare) return target
    let file = bare.startsWith('/')
      ? posix.normalize(bare.slice(1))
      : posix.normalize(posix.join(posix.dirname(srcRel), decodeURI(bare)))
    file = file.replace(/\/$/, '')
    if (isDir(file) && bySource.has(`${file}/README.md`)) file = `${file}/README.md`
    if (bySource.has(file)) return relativeLink(destRel, bySource.get(file) + '.md') + hash
    if (byFile.has(file)) return `/files/${byFile.get(file)}`
    if (IMAGE.test(file) && exists(file)) return relativeLink(destRel, copyAsset(file)) + hash
    const kind = isDir(file) ? 'tree' : 'blob'
    return `${repoUrl}/${kind}/${branch}/${file}${hash}`
  }
}

export function rewriteLinks(line, link, srcRel, destRel) {
  return line
    .replace(/\]\(\s*<?([^)\s>]+)>?((?:\s+"[^"]*")?)\s*\)/g,
      (_, target, title) => `](${link(target, srcRel, destRel)}${title})`)
    .replace(/\b(src|href)="([^"]+)"/g, (_, attr, target) => `${attr}="${link(target, srcRel, destRel)}"`)
    .replace(/^(\s*\[[^\]]+\]:\s*)(\S+)/, (_, head, target) => head + link(target, srcRel, destRel))
}

// ---------------------------------------------------------------- one page

export function transform(text, { src, dest, link, header, extraFrontMatter }) {
  const { yaml, body } = splitFrontMatter(text)
  let lines = admonitions(body.split('\n'))
  let fence = null
  lines = lines.map((line) => {
    const f = /^\s*(```+|~~~+)/.exec(line)
    if (fence) {
      if (f && f[1][0] === fence[0] && f[1].length >= fence.length) fence = null
      return line
    }
    if (f) {
      fence = f[1]
      return line
    }
    line = line.replace(/\{:\s*([^}]*?)\s*\}/g, '{$1}')
    return outsideInlineCode(line, (part) => escapeForVue(rewriteLinks(part, link, src, dest)))
  })
  const meta = scalars(yaml)
  const front = [yaml, ...Object.entries(extraFrontMatter ?? {}).map(([k, v]) => `${k}: ${JSON.stringify(v)}`)]
    .filter(Boolean)
    .join('\n')
  const head = header ? header(meta) : ''
  return `${front ? `---\n${front}\n---\n` : ''}${head}${lines.join('\n')}`
}

// A title block for a versioned guide: its title, subtitle, audience, version and PDF.
function guideHeader(item) {
  return (meta) => {
    if (!meta.version) return ''
    const pdf = item.pdf ? `**Download:** [the printable PDF](/files/${item.pdf})${item.pdfNote ? ', ' + item.pdfNote : ''}.` : ''
    const out = [`<h1 class="doc-title">${meta.title ?? item.text}</h1>\n`]
    if (meta.subtitle) out.push(`<p class="doc-subtitle">${meta.subtitle}</p>\n`)
    out.push(`::: info Version ${meta.version}${meta.date ? ' · ' + meta.date : ''}`)
    if (meta.audience) out.push(meta.audience)
    if (meta.note) out.push('', meta.note)
    if (pdf) out.push('', pdf)
    out.push(':::\n\n')
    return out.join('\n')
  }
}

// ---------------------------------------------------------------- main

function copyTree(from, to) {
  if (!fs.existsSync(from)) return
  fs.cpSync(from, to, { recursive: true })
}

function main() {
  fs.rmSync(OUT, { recursive: true, force: true })
  fs.mkdirSync(OUT, { recursive: true })
  const pages = pageList().map((p) => ({ ...p, dest: p.dest.replace(/\.md$/, '') }))
  const files = site.files ?? []
  const repoUrl = `https://github.com/${site.repo}`
  const exists = (rel) => fs.existsSync(path.join(ROOT, rel))
  const isDir = (rel) => exists(rel) && fs.statSync(path.join(ROOT, rel)).isDirectory()
  const copyAsset = (rel) => {
    const to = posix.join('assets', rel.replace(/^docs\//, ''))
    fs.mkdirSync(path.dirname(path.join(OUT, to)), { recursive: true })
    fs.copyFileSync(path.join(ROOT, rel), path.join(OUT, to))
    return to
  }
  const link = makeLinker({ pages, files, repoUrl, branch: site.branch ?? 'main', exists, isDir, copyAsset })

  for (const item of pages) {
    let text = fs.readFileSync(path.join(ROOT, item.src), 'utf8')
    for (const r of site.replacements ?? []) {
      if (r.src !== item.src || !text.includes(r.find)) continue
      const output = execSync(r.command, { cwd: ROOT, encoding: 'utf8', maxBuffer: 64 << 20 })
      text = text.replace(r.find, () => output)
    }
    const extra = { source: item.src, ...(item.frontmatter ?? {}) }
    if (item.numbered) extra.pageClass = 'numbered'
    const page = transform(text, {
      src: item.src,
      dest: item.dest + '.md',
      link,
      header: guideHeader(item),
      extraFrontMatter: extra,
    })
    const target = path.join(OUT, item.dest + '.md')
    fs.mkdirSync(path.dirname(target), { recursive: true })
    fs.writeFileSync(target, page)
  }
  for (const f of files) {
    const target = path.join(OUT, 'public', 'files', f.as)
    fs.mkdirSync(path.dirname(target), { recursive: true })
    fs.copyFileSync(path.join(ROOT, f.src), target)
  }
  copyTree(path.join(SITE, 'public'), path.join(OUT, 'public'))
  fs.copyFileSync(path.join(SITE, 'index.md'), path.join(OUT, 'index.md'))
  console.log(`site: ${pages.length} pages, ${files.length} downloads -> ${path.relative(ROOT, OUT)}`)
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) main()
