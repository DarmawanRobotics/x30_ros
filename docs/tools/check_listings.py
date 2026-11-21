#!/usr/bin/env python3
"""The commands of the documents, copied whole: none wraps, none carries a comment after it.

A line wider than its page wraps in the PDF, and copied it runs as two commands; a comment
after a command breaks the same way on a narrow screen (its words run as a command of their
own), and after a value in a settings file systemd keeps it in the value. Every bash listing
under latex/ is checked: no line past the page's width, no ``#`` after code on a line.

    python3 tools/check_listings.py          exit 1 and every line at fault, else nothing
"""

from __future__ import annotations

from pathlib import Path
import re
import sys

LATEX = Path(__file__).resolve().parents[1] / 'latex'
# characters a listing line holds before it wraps: \small DejaVu Sans Mono at 0.84 within the
# text width, less the frame (x30doc.cls); the A5 user manual is narrower
WIDTH = {'user-manual': 72}
DEFAULT_WIDTH = 88
LISTING = re.compile(r'\\begin\{lstlisting\}(\[[^\]]*\])?\n(.*?)\\end\{lstlisting\}', re.S)
TRAILING_COMMENT = re.compile(r'^\s*[^#\s].*\s#(\s|$)')


def faults(tex: Path, width: int) -> list[str]:
    """Return 'file:line: what: text' for each listing line at fault in tex."""
    text = tex.read_text()
    found = []
    for match in LISTING.finditer(text):
        if 'bash' not in (match.group(1) or ''):
            continue
        first = text.count('\n', 0, match.start(2)) + 1
        for number, line in enumerate(match.group(2).splitlines(), first):
            what = []
            if TRAILING_COMMENT.search(line):
                what.append('a comment after the command')
            if len(line) > width:
                what.append(f'{len(line)} characters, the page holds {width}')
            if what:
                where = f'{tex.relative_to(LATEX.parent)}:{number}'
                found.append(f'{where}: {", ".join(what)}: {line}')
    return found


def main() -> int:
    """Print every listing line at fault; exit 1 when there is one."""
    found = []
    for tex in sorted(LATEX.rglob('*.tex')):
        document = tex.relative_to(LATEX).parts[0].split('.')[0]
        found += faults(tex, WIDTH.get(document, DEFAULT_WIDTH))
    for line in found:
        print(line)
    return 1 if found else 0


if __name__ == '__main__':
    sys.exit(main())
