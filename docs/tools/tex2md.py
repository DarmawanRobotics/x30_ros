"""The website's pages from the LaTeX documents: one page per chapter, one landing page per document.

python3 docs/tools/tex2md.py        (make -C docs site runs it)

The LaTeX stays the one source. Each chapter of the five documents is prepared here (the class's
macros as plain LaTeX, verbatim code, numbered equations, cross references as links, the boxes
with their titles) and converted by pandoc with tex2md.lua into GitHub Markdown with $math$ for
VitePress. Output (git-ignored): docs/site/generated/<document>/<chapter>.md, a landing page per
document and pages.json, the sections and downloads site.config.mjs reads. Needs pandoc 3.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

DOCS = Path(__file__).resolve().parents[1]
LATEX = DOCS / 'latex'
OUT = DOCS / 'site' / 'generated'
FILTER = Path(__file__).with_suffix('.lua')

# document, its PDF, its short name in the navigation
DOCUMENTS = [
    ('installation-guide', 'X30-Installation-Guide.pdf', 'Installation'),
    ('user-manual', 'X30-User-Manual.pdf', 'User Manual'),
    ('tuning-guide', 'X30-Tuning-Guide.pdf', 'Tuning'),
    ('architecture', 'X30-Architecture.pdf', 'Architecture'),
    ('acceptance-test', 'X30-Site-Acceptance-Test.pdf', 'Acceptance Test'),
]

# the class's macros as plain LaTeX for pandoc (code macros become \verb before pandoc sees them)
MACROS = r"""
\newcommand{\ui}[1]{\textbf{#1}}
\newcommand{\menupath}[1]{\textbf{#1}}
\newcommand{\key}[1]{\textbf{#1}}
\newcommand{\vect}[1]{\boldsymbol{#1}}
\newcommand{\mat}[1]{\mathbf{#1}}
\newcommand{\T}[2]{{}^{#1}\mathbf{T}_{#2}}
\newcommand{\R}{\mathbb{R}}
\newcommand{\textdegree}{°}
\newcommand{\checkbox}{☐}
\newcommand{\result}{☐ Pass ☐ Fail}
\newcommand{\blank}[1][]{\_\_\_\_\_\_}
\newcommand{\readonlynote}{Parameters marked $^\dagger$ are read at start only; every other one
  can be changed while the node runs (\verb|rosparam set|) and is checked before it applies.}
"""
CODE_MACROS = ('cmd', 'code', 'pkg', 'node', 'topic', 'srv', 'param', 'file')
BOXES = {'notebox': 'Note', 'tipbox': 'Tip', 'warnbox': 'Warning', 'dangerbox': 'Danger'}
# the SAT's test tables (acceptance-test.tex's tests environment)
TESTS_BEGIN = (r'\begin{longtable}{llll}\toprule \textbf{ID} & \textbf{Procedure} & '
               r'\textbf{Pass when} & \textbf{Result} \\ \midrule \endhead \bottomrule \endfoot')
TESTS_END = r'\end{longtable}'


def braced(text: str, start: int) -> int:
    """Return the index after the group opened by text[start] == '{'."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == '\\':
            continue
        if text[i] == '{' and (i == 0 or text[i - 1] != '\\'):
            depth += 1
        elif text[i] == '}' and (i == 0 or text[i - 1] != '\\'):
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError(f'unbalanced braces from {text[start:start + 40]!r}')


def verbatim(text: str) -> str:
    """\\code{x} and the other code macros as \\verb, so _ ~ { } < > stay as they are."""
    pattern = re.compile(r'\\(' + '|'.join(CODE_MACROS) + r')\{')
    out, pos = [], 0
    while (m := pattern.search(text, pos)) is not None:
        end = braced(text, m.end() - 1)
        body = ' '.join(text[m.end():end - 1].split())
        if m.group(1) == 'cmd':  # \cmd's argument is detokenized: \_ was never written
            body = body.replace('\\\\', '\\')
        delim = next(c for c in '|!+=@;:^' if c not in body)
        out += [text[pos:m.start()], f'\\verb{delim}{body}{delim}']
        pos = end
    return ''.join(out + [text[pos:]])


def plain_argument(text: str, name: str) -> str | None:
    """The argument of \\name{...} in text, or None."""
    m = re.search(r'\\' + name + r'\{', text)
    return None if m is None else text[m.end():braced(text, m.end() - 1) - 1]


class Chapter:
    """One chapter file: its number, title, label and where it is on the site."""

    def __init__(self, doc: str, stem: str, number: str):
        self.doc, self.stem, self.number = doc, stem, number
        self.path = LATEX / doc / f'{stem}.tex'
        self.text = self.path.read_text()
        self.title = plain_argument(self.text, 'chapter') or stem

    @property
    def page(self) -> str:
        return f'{self.doc}/{self.stem}.md'


def chapters_of(doc: str) -> list[Chapter]:
    """The chapters of a document in order, numbered 1, 2, ... and A, B, ... after \\appendix."""
    main = (LATEX / f'{doc}.tex').read_text()
    out, numbers, letters = [], iter(range(1, 100)), iter('ABCDEFGH')
    appendix = False
    for line in main.splitlines():
        if line.strip().startswith('\\appendix'):
            appendix = True
        m = re.match(r'\s*\\input\{' + re.escape(doc) + r'/([\w-]+)\}', line)
        if m:
            number = next(letters) if appendix else str(next(numbers))
            out.append(Chapter(doc, m.group(1), number))
    return out


def anchor(label: str) -> str:
    """A URL anchor from a LaTeX label."""
    return re.sub(r'[^a-z0-9-]+', '-', label.lower()).strip('-')


def collect(chapters: list[Chapter]) -> dict[str, tuple[str, str, str]]:
    """label -> (number, page, anchor) of every chapter, section and equation."""
    refs = {}
    for ch in chapters:
        lines = ch.text.splitlines()
        section, sub, equation = 0, 0, 0
        last = ('chapter', ch.number, '')
        for i, line in enumerate(lines):
            if re.match(r'\s*\\chapter\{', line):
                last = ('chapter', ch.number, '')
            elif m := re.match(r'\s*\\section\{', line):
                section, sub = section + 1, 0
                last = ('section', f'{ch.number}.{section}', '')
            elif re.match(r'\s*\\subsection\{', line):
                sub += 1
                last = ('subsection', f'{ch.number}.{section}.{sub}', '')
            for label in re.findall(r'\\label\{([^}]+)\}', line):
                if label.startswith('eq:'):
                    continue
                kind, number, _ = last
                refs[label] = (number, ch.page, '' if kind == 'chapter' else anchor(label))
        # equations: numbered per chapter, in order (an align row each)
        for m in re.finditer(r'\\begin\{(equation|align)\}(.*?)\\end\{\1\}', ch.text, re.S):
            rows = m.group(2).split('\\\\') if m.group(1) == 'align' else [m.group(2)]
            for row in rows:
                equation += 1
                for label in re.findall(r'\\label\{([^}]+)\}', row):
                    refs[label] = (f'({ch.number}.{equation})', ch.page, '')
    return refs


def relative(from_page: str, to_page: str) -> str:
    """The relative link between two generated pages."""
    a, b = Path(from_page).parent, Path(to_page)
    return str(Path('..') / b) if a != b.parent else b.name


UNITS = {'\\celsius': '{}^\\circ\\mathrm{C}', '\\percent': '\\%', '\\degree': '{}^\\circ'}


def si_in_math(math: str) -> str:
    r"""siunitx inside math, which MathJax does not know: \SI{0.5}{m/s} as 0.5\,\mathrm{m/s}."""
    def unit(u: str) -> str:
        for k, v in UNITS.items():
            if k in u:
                return u.replace(k, v)
        return '\\mathrm{' + u + '}'

    math = re.sub(r'\\SI\{([^}]*)\}\{([^}]*)\}', lambda m: m.group(1) + '\\,' + unit(m.group(2)), math)
    math = re.sub(r'\\ang\{([^}]*)\}', lambda m: m.group(1) + '^\\circ', math)
    return re.sub(r'\\num\{([^}]*)\}', lambda m: m.group(1), math)


def link_refs(text: str, page: str, refs: dict) -> str:
    """Cross references as links: Chapter 4, Section 4.3, Eq. (2.1), Appendix A."""
    def ref(m: re.Match) -> str:
        label = m.group(1)
        if label not in refs:
            print(f'{page}: unknown reference {label}', file=sys.stderr)
            return ' ??'
        number, target, hash_ = refs[label]
        href = relative(page, target) + (f'#{hash_}' if hash_ else '')
        return f' \\href{{{href}}}{{{number}}}'

    return re.sub(r'~?\\ref\{([^}]+)\}', ref, text)


def prepare(ch: Chapter, refs: dict) -> str:
    """The chapter's LaTeX made plain for pandoc."""
    text = re.sub(r'(?<!\\)\$([^$]+)\$', lambda m: '$' + si_in_math(m.group(1)) + '$', ch.text)
    # equations: \tag the number of the PDF, one equation per align row
    counter = iter(range(1, 1000))

    def number(m: re.Match) -> str:
        rows = m.group(2).split('\\\\') if m.group(1) == 'align' else [m.group(2)]
        out = []
        for row in rows:
            body = re.sub(r'\\label\{[^}]+\}', '', row).replace('&', '').strip()
            out.append(f'\\begin{{equation}}{body}\\tag{{{ch.number}.{next(counter)}}}\\end{{equation}}')
        return '\n'.join(out)

    text = re.sub(r'\\begin\{(equation|align)\}(.*?)\\end\{\1\}', number, text, flags=re.S)
    text = link_refs(text, ch.page, refs)
    # headings keep their label as the anchor: \label right after them
    text = re.sub(r'(\\(?:sub)?section\{)', r'\n\1', text)
    # boxes: the optional title as a first block pandoc keeps
    for box, default in BOXES.items():
        text = re.sub(r'\\begin\{' + box + r'\}(?:\[([^\]]*)\])?',
                      lambda m, d=default, b=box: f'\\begin{{{b}}}\\begin{{boxtitle}}{m.group(1) or d}'
                                                  f'\\end{{boxtitle}}', text)
    text = text.replace('\\begin{tests}', TESTS_BEGIN).replace('\\end{tests}', TESTS_END)
    text = re.sub(r'\\qtyproduct\{([^}]*)\}\{([^}]*)\}',
                  lambda m: m.group(1).replace(' x ', ' × ') + ' ' + m.group(2), text)
    text = text.replace('\\begin{enumerate}[resume]', '\\begin{enumerate}')
    text = text.replace('\\newline', ' ')
    return MACROS + verbatim(text)


def pandoc(latex: str, chapter: str = '') -> str:
    """LaTeX to GitHub Markdown with $math$, through the filter (chapter: numbers the headings)."""
    run = subprocess.run(
        ['pandoc', '-f', 'latex', '-t', 'gfm+tex_math_dollars', '--wrap=none',
         '--lua-filter', str(FILTER), '-M', f'chapter={chapter}'],
        input=latex, capture_output=True, text=True, check=False)
    if run.returncode != 0:
        raise RuntimeError(run.stderr)
    # <placeholders> as entities: the site's sync escapes what looks like a tag, not entities
    return run.stdout.replace('\\<', '&lt;').replace('\\>', '&gt;')


def metadata(doc: str, refs: dict) -> dict[str, str]:
    """Title, subtitle, version, date, audience and note of a document, as Markdown."""
    main = (LATEX / f'{doc}.tex').read_text()
    out = {}
    for key in ('doctitle', 'docsubtitle', 'docversion', 'docdate', 'docaudience', 'docnote'):
        value = plain_argument(main, key)
        if value is not None:
            value = link_refs(' '.join(value.split()), f'{doc}/index.md', refs)
            md = pandoc(MACROS + verbatim(value)).strip()
            out[key[3:]] = ' '.join(md.split())
    return out


def front(fields: dict[str, str]) -> str:
    """A YAML front matter of plain scalars."""
    return '---\n' + ''.join(f'{k}: {json.dumps(v, ensure_ascii=False)}\n' for k, v in fields.items()) + '---\n'


def main():
    if shutil.which('pandoc') is None:
        sys.exit('pandoc is needed (apt install pandoc)')
    shutil.rmtree(OUT, ignore_errors=True)
    every = {doc: chapters_of(doc) for doc, _, _ in DOCUMENTS}
    refs = collect([ch for chapters in every.values() for ch in chapters])
    sections, files, count = [], [], 0
    for doc, pdf, short in DOCUMENTS:
        meta = metadata(doc, refs)
        title = meta.get('title', doc)
        items = [{'src': f'docs/site/generated/{doc}/index.md', 'dest': f'{doc}/index',
                  'text': 'Overview', 'pdf': pdf, 'frontmatter': {'source': f'docs/latex/{doc}.tex'}}]
        if doc == 'user-manual':
            items[0]['pdfNote'] = 'A5 for the phone'
        listing = []
        for ch in every[doc]:
            body = pandoc(prepare(ch, refs), ch.number)
            (OUT / doc).mkdir(parents=True, exist_ok=True)
            fields = {'title': ch.title}
            heading = f'# {ch.number}. ' if ch.number.isdigit() else f'# Appendix {ch.number}. '
            body = re.sub(r'^# ', heading, body, count=1, flags=re.M)
            (OUT / ch.page).write_text(front(fields) + '\n' + body)
            label = f'{ch.number}. {ch.title}'
            items.append({'src': f'docs/site/generated/{ch.page}', 'dest': f'{doc}/{ch.stem}',
                          'text': label,
                          'frontmatter': {'source': f'docs/latex/{doc}/{ch.stem}.tex'}})
            kind = '' if ch.number.isdigit() else 'Appendix '
            listing.append(f'- **{kind}{ch.number}.** [{ch.title}]({ch.stem}.md)')
            count += 1
        landing = {'title': title, 'subtitle': meta.get('subtitle', ''),
                   'version': meta.get('version', ''), 'date': meta.get('date', ''),
                   'audience': meta.get('audience', ''), 'note': meta.get('note', '')}
        body = '## Chapters\n\n' + '\n'.join(listing) + '\n'
        (OUT / doc / 'index.md').write_text(front(landing) + '\n' + body)
        sections.append({'text': title, 'short': short, 'items': items})
        files.append({'src': f'docs/pdf/{pdf}', 'as': pdf, 'text': title})
    (OUT / 'pages.json').write_text(json.dumps({'sections': sections, 'files': files}, indent=1))
    print(f'tex2md: {count} chapters of {len(DOCUMENTS)} documents -> {OUT.relative_to(DOCS.parent)}')


if __name__ == '__main__':
    main()
