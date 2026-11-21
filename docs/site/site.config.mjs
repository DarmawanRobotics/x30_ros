// The X30 documentation site: its pages are generated from the LaTeX documents of docs/latex by
// docs/tools/tex2md.py (one page per chapter, a landing page per document, generated/pages.json);
// scripts/sync.mjs copies them into content/. Run `make -C docs site` for the whole chain.
import fs from 'node:fs'

const generated = new URL('./generated/pages.json', import.meta.url)
const pages = fs.existsSync(generated)
  ? JSON.parse(fs.readFileSync(generated, 'utf8'))
  : { sections: [], files: [] } // python3 docs/tools/tex2md.py first

export default {
  title: 'X30',
  description:
    'The Jueying X30 substation patrol robot of EZRA Robotics: installation, operation, tuning, ' +
    'architecture and site acceptance of the x30_ros stack, the X30Controller handheld and NETRA.',
  repo: 'DarmawanRobotics/x30_ros',
  branch: 'main',
  logo: 'logo.svg',
  brand: { light: '#1F6FB2', light2: '#2f82c8', dark: '#7fb6e6', dark2: '#9cc7ee', soft: 'rgba(31,111,178,.16)' },
  copyright: 'Apache-2.0 · EZRA Robotics',
  footer: 'Generated from the LaTeX documents of <a href="https://github.com/DarmawanRobotics/x30_ros/tree/main/docs/latex">DarmawanRobotics/x30_ros</a>.',
  math: true,
  sidebarPerSection: true,
  sections: pages.sections.map((s) => ({ text: s.short, title: s.text, items: s.items })),
  files: pages.files,
}
