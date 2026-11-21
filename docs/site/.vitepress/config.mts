// The site of this repository's documentation, generic: everything specific is in
// site.config.mjs. Pages come from scripts/sync.mjs (content/), which `npm run build` runs first.
import { defineConfig } from 'vitepress'

import site from '../site.config.mjs'

const base = process.env.DOCS_BASE || '/'
const branch = site.branch ?? 'main'
const repoUrl = `https://github.com/${site.repo}`
const link = (dest: string) => '/' + dest.replace(/\.md$/, '').replace(/(^|\/)index$/, '$1')
const brand = site.brand ?? { light: '#3451b2', dark: '#a8b1ff' }

export default defineConfig({
  title: site.title,
  description: site.description,
  lang: 'en-US',
  base,
  srcDir: 'content',
  cleanUrls: true,
  lastUpdated: false,
  ignoreDeadLinks: 'localhostLinks', // the dev container's VNC and the like
  head: [
    ['link', { rel: 'icon', href: base + site.logo }],
    ['meta', { name: 'theme-color', content: brand.light }],
    ['meta', { property: 'og:title', content: site.title }],
    ['meta', { property: 'og:description', content: site.description }],
    [
      'style',
      {},
      `:root{--vp-c-brand-1:${brand.light};--vp-c-brand-2:${brand.light2 ?? brand.light};` +
        `--vp-c-brand-3:${brand.light};--vp-c-brand-soft:${brand.soft ?? 'rgba(100,108,255,.14)'}}` +
        `.dark{--vp-c-brand-1:${brand.dark};--vp-c-brand-2:${brand.dark2 ?? brand.dark};` +
        `--vp-c-brand-3:${brand.dark}}`,
    ],
  ],
  markdown: {
    math: site.math ?? false, // $...$ and $$...$$ through MathJax (markdown-it-mathjax3)
    config(md) {
      // inline code may hold {{ }} (Vue would compile it)
      const code = md.renderer.rules.code_inline!
      md.renderer.rules.code_inline = (tokens, idx, options, env, self) =>
        code(tokens, idx, options, env, self).replace(/^<code/, '<code v-pre')
      // a paragraph holding only an image is a figure, its alt text the caption (as in the PDFs)
      md.core.ruler.push('figures', (state) => {
        const t = state.tokens
        for (let i = 0; i + 2 < t.length; i++) {
          if (t[i].type !== 'paragraph_open' || t[i + 1].type !== 'inline') continue
          const kids = (t[i + 1].children ?? []).filter((k) => k.type !== 'text' || k.content.trim())
          if (kids.length !== 1 || kids[0].type !== 'image' || !kids[0].content) continue
          t[i].tag = t[i + 2].tag = 'figure'
          const caption = new state.Token('html_inline', '', 0)
          caption.content = `<figcaption>${md.utils.escapeHtml(kids[0].content)}</figcaption>`
          t[i + 1].children!.push(caption)
        }
      })
    },
  },
  themeConfig: {
    logo: '/' + site.logo,
    siteTitle: site.title,
    nav: [
      ...site.sections
        .filter((section) => section.nav !== false)
        .map((section) => ({
          text: section.text,
          link: link(section.items[0].dest),
          activeMatch: `^/${section.items[0].dest.split('/')[0]}/`,
        })),
      ...(site.files?.some((f) => f.text)
        ? [
            {
              text: 'PDF',
              items: site.files
                .filter((f) => f.text)
                .map((f) => ({ text: f.text, link: `${base}files/${f.as}`, target: '_blank' })),
            },
          ]
        : []),
    ],
    // one sidebar for all sections, or (sidebarPerSection) each section's own under its folder
    sidebar: site.sidebarPerSection
      ? Object.fromEntries(
          site.sections.map((section) => [
            `/${section.items[0].dest.split('/')[0]}/`,
            [
              {
                text: section.title ?? section.text,
                items: section.items.map((item) => ({ text: item.text, link: link(item.dest) })),
              },
            ],
          ])
        )
      : site.sections.map((section) => ({
          text: section.text,
          collapsed: section.collapsed ?? false,
          items: section.items.map((item) => ({ text: item.text, link: link(item.dest) })),
        })),
    outline: { level: [2, 3], label: 'On this page' },
    socialLinks: [{ icon: 'github', link: repoUrl }],
    search: { provider: 'local' },
    editLink: {
      // serialized for the browser without its closure: the values are written into it
      pattern: new Function(
        'page',
        `return page.frontmatter.source ? ${JSON.stringify(`${repoUrl}/edit/${branch}/`)} + ` +
          `page.frontmatter.source : ${JSON.stringify(repoUrl)}`
      ) as (page: { frontmatter: Record<string, string> }) => string,
      text: 'Edit this page on GitHub',
    },
    footer: {
      message: site.footer ?? `The pages are the Markdown of <a href="${repoUrl}">${site.repo}</a>.`,
      copyright: site.copyright,
    },
  },
})
