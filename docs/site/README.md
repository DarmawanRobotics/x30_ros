# Documentation site

The five X30 documents as a website (VitePress): one page per chapter, search, dark mode, the
equations (MathJax), the figures and the PDFs to download. The LaTeX in [`docs/latex`](../latex)
stays the one source: [`docs/tools/tex2md.py`](../tools/tex2md.py) turns it into the pages, the
site is built from them and published to GitHub Pages by
[`.github/workflows/docs.yml`](../../.github/workflows/docs.yml).

```bash
make -C docs site                 # figures, PDFs, pages, site: docs/site/.vitepress/dist
cd docs/site && npm run dev       # http://localhost:5173 after one make; python3 ../tools/tex2md.py
                                  # again after editing a chapter
DOCS_BASE=/x30_ros/ npm run build # the GitHub Pages path
```

| File | Holds |
|---|---|
| `../tools/tex2md.py`, `../tools/tex2md.lua` | the LaTeX of each chapter prepared (the class's macros, verbatim code, numbered equations, cross references as links, the boxes' titles) and converted by pandoc 3; output in `generated/` (git-ignored) with `pages.json`, the sections and downloads |
| `site.config.mjs` | this site: title, colours, math, a sidebar per document, from `generated/pages.json` |
| `index.md`, `public/` | the home page and the logo; `make figures` puts the figures' SVGs in `public/figures` |
| `scripts/sync.mjs` | copies the pages into `content/` (git-ignored), fixing links, images and boxes |
| `.vitepress/config.mts`, `.vitepress/theme/` | the VitePress configuration and look (the Dharma template's, with math and a sidebar per section) |

A page's "Edit this page" opens its LaTeX chapter on GitHub. Needs pandoc 3, Node 20 and, for the
figures and the PDFs, the TeX Live of [`docs/README.md`](../README.md).

**Publishing.** Once per repository: Settings → Pages → Source: **GitHub Actions**. Until then the
workflow builds the site on every docs change and only reports that Pages is off.
