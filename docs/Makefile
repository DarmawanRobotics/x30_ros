# X30 documentation: the LaTeX documents, their figures, and the website built from them.
#
#   make              the figures, then every document (build/*.pdf)
#   make pdf          every document, copied to pdf/ (the committed PDFs: commit them after a change)
#   make figures      latex/figures/*.tex -> build/figures/*.pdf, and SVG for the website
#   make reference    latex/generated/params_*.tex from the X30 sources (tools/params.py)
#   make site         the website in site/.vitepress/dist (pages from the LaTeX, the PDFs)
#   make clean        remove build/
#
# Needs XeLaTeX with fontspec, unicode-math, tcolorbox, pgfplots, siunitx and qrcode (TeX Live
# 2022 or newer), latexmk, pdftocairo (poppler) for the SVG figures; pandoc 3 and Node 20 for
# the website. Everything compiles from latex/, where the class, the fonts and the figures are.

LATEXMK ?= latexmk
LATEXFLAGS ?= -xelatex -interaction=nonstopmode -halt-on-error -file-line-error
PYTHON ?= python3

DOCS := installation-guide user-manual tuning-guide architecture acceptance-test
NAME_installation-guide := X30-Installation-Guide
NAME_user-manual := X30-User-Manual
NAME_tuning-guide := X30-Tuning-Guide
NAME_architecture := X30-Architecture
NAME_acceptance-test := X30-Site-Acceptance-Test

FIGURES := $(notdir $(basename $(wildcard latex/figures/*.tex)))
FIGURE_PDFS := $(addprefix build/figures/,$(addsuffix .pdf,$(FIGURES)))
FIGURE_SVGS := $(addprefix site/public/figures/,$(addsuffix .svg,$(FIGURES)))
COMMON := latex/x30doc.cls latex/assets/logo.pdf $(wildcard latex/generated/*.tex)

.PHONY: all figures docs pdf reference site clean

all: figures docs

figures: $(FIGURE_PDFS) $(FIGURE_SVGS)

build/figures/%.pdf: latex/figures/%.tex latex/x30fig.sty
	cd latex && $(LATEXMK) $(LATEXFLAGS) -outdir=../build/figures figures/$*.tex

site/public/figures/%.svg: build/figures/%.pdf
	@mkdir -p $(@D)
	pdftocairo -svg $< $@

latex/assets/logo.pdf: latex/assets/logo.svg
	rsvg-convert -f pdf -o $@ $<

docs: $(addprefix build/,$(addsuffix .pdf,$(DOCS)))

.SECONDEXPANSION:
build/%.pdf: latex/%.tex $$(wildcard latex/%/*.tex) $(COMMON) $(FIGURE_PDFS)
	cd latex && $(LATEXMK) $(LATEXFLAGS) -outdir=../build $*.tex

pdf: docs
	@mkdir -p pdf
	@$(foreach d,$(DOCS),cp build/$(d).pdf pdf/$(NAME_$(d)).pdf;)
	@ls -1 pdf

reference:
	$(PYTHON) tools/params.py

site: figures
	cd site && npm ci --no-audit --no-fund && npm run build

clean:
	rm -rf build
