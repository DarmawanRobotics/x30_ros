% x30doc: the documents of the X30 stack (installation guide, user manual, tuning guide,
% architecture, acceptance test). XeLaTeX or LuaLaTeX, run from docs/latex (make -C docs).
%
%   \documentclass{x30doc}        A4, the technical documents
%   \documentclass[a5]{x30doc}    A5, readable on a phone or a tablet in the field
%
% The look follows the EZRA documents of HotDog and Pashupati: a night-blue cover with the brand
% in gold, Barlow Semi Condensed, blue accents, coloured boxes for notes, tips and warnings.
% Everything the body uses also reads through pandoc (tools/tex2md.py builds the website from
% the same sources), so keep to the macros and environments defined here.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{x30doc}[2026/11/21 X30 documents]

\newif\ifxdoc@afive
\DeclareOption{a5}{\xdoc@afivetrue}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions\relax
\ifxdoc@afive
  \LoadClass[10pt,oneside]{report}
\else
  \LoadClass[11pt,oneside]{report}
\fi

% ----------------------------------------------------------------------------------- page
\ifxdoc@afive
  \RequirePackage[a5paper,top=17mm,bottom=16mm,left=13mm,right=13mm,headheight=13pt,
    headsep=5mm,footskip=8mm]{geometry}
\else
  \RequirePackage[a4paper,top=26mm,bottom=24mm,left=24mm,right=24mm,headheight=14pt,
    headsep=7mm,footskip=11mm]{geometry}
\fi

% ----------------------------------------------------------------------------------- fonts
\RequirePackage{amsmath}
\RequirePackage{fontspec}
\RequirePackage{unicode-math}
\defaultfontfeatures{Ligatures=TeX}
\setmainfont{BarlowSemiCondensed}[
  Path=fonts/, Extension=.ttf,
  UprightFont=*-Regular, BoldFont=*-SemiBold, ItalicFont=*-Italic, BoldItalicFont=*-Italic]
\setsansfont{BarlowSemiCondensed}[
  Path=fonts/, Extension=.ttf,
  UprightFont=*-Regular, BoldFont=*-SemiBold, ItalicFont=*-Italic, BoldItalicFont=*-Italic]
\newfontfamily\xheavy{BarlowSemiCondensed}[Path=fonts/, Extension=.ttf, UprightFont=*-Bold]
\newfontfamily\xmedium{BarlowSemiCondensed}[Path=fonts/, Extension=.ttf, UprightFont=*-Medium]
% one space after a full stop or colon too, as sans-serif manuals set it
\AtBeginDocument{\frenchspacing}
% word spaces of the big titles, after their \selectfont (em of the title's size)
\newcommand{\titlespace}{\spaceskip=0.27em plus 0.08em minus 0.04em\relax\xspaceskip=\spaceskip}
\IfFontExistsTF{DejaVu Sans Mono}
  {\setmonofont{DejaVu Sans Mono}[Scale=0.84]}
  {\setmonofont{lmmono10-regular.otf}}
\IfFontExistsTF{DejaVu Math TeX Gyre}
  {\setmathfont{DejaVu Math TeX Gyre}[Scale=0.94]}
  {\setmathfont{latinmodern-math.otf}}

% ----------------------------------------------------------------------------------- colours
\RequirePackage[table]{xcolor}
\definecolor{ink}{HTML}{16212A}
\definecolor{muted}{HTML}{5B6B75}
\definecolor{accent}{HTML}{1F6FB2}
\definecolor{accentsoft}{HTML}{E6EFF8}
\definecolor{orange}{HTML}{E8892C}
\definecolor{orangesoft}{HTML}{FDF0E2}
\definecolor{red}{HTML}{D64545}
\definecolor{redsoft}{HTML}{FBE9E9}
\definecolor{green}{HTML}{2E8B57}
\definecolor{greensoft}{HTML}{E5F4EA}
\definecolor{line}{HTML}{D5DDE2}
\definecolor{night}{HTML}{0F2742}
\definecolor{nightsoft}{HTML}{1C3A5E}
\definecolor{gold}{HTML}{FFB020}
\definecolor{coversub}{HTML}{C5D3E6}
\definecolor{coverfoot}{HTML}{8FA1A8}
\definecolor{codeback}{HTML}{F4F7F9}
\color{ink}

% ----------------------------------------------------------------------------------- packages
\RequirePackage{graphicx}
\RequirePackage{booktabs,longtable,tabularx,array,multirow}
% X columns ragged right: justified narrow cells open wide gaps
\renewcommand{\tabularxcolumn}[1]{>{\raggedright\arraybackslash}p{#1}}
\RequirePackage{enumitem}
\RequirePackage{caption}
\RequirePackage{titlesec}
\RequirePackage{fancyhdr}
\RequirePackage{lastpage}
\RequirePackage{listings}
\RequirePackage{tikz}
\usetikzlibrary{calc,positioning,arrows.meta,shapes.geometric,fit,backgrounds}
\RequirePackage[most]{tcolorbox}
\RequirePackage{siunitx}
\RequirePackage{qrcode}
\RequirePackage{xurl}
\RequirePackage{pdfpages}
\RequirePackage{hyperref}
\hypersetup{colorlinks=true,linkcolor=accent,urlcolor=accent,citecolor=accent,
  bookmarksnumbered=true,bookmarksopen=true,pdfcreator={x30doc (XeLaTeX)}}

\graphicspath{{../build/figures/}{assets/}}
\sisetup{per-mode=symbol,group-minimum-digits=4,detect-all}

% ----------------------------------------------------------------------------------- layout
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.55em plus 0.15em}
\setlength{\emergencystretch}{3em}
\renewcommand{\arraystretch}{1.22}
\setlist{itemsep=1pt,topsep=3pt,parsep=1pt,leftmargin=1.4em}
\setlist[enumerate]{label=\textcolor{accent}{\bfseries\arabic*.}}
\setlist[itemize]{label=\textcolor{accent}{\textbullet}}
\captionsetup{font={small,color=muted},labelfont={bf,color=ink},skip=4pt}
\captionsetup[table]{position=top}
\arrayrulecolor{line}
\setlength{\aboverulesep}{0.3ex}
\setlength{\belowrulesep}{0.5ex}
\clubpenalty=8000
\widowpenalty=8000

% headings: the chapter number in the accent, the title heavy, a thin rule under it
\titleformat{\chapter}[display]
  {\xheavy\color{ink}}
  {\xmedium\large\color{accent}\MakeUppercase{\chaptertitlename}~\thechapter}
  {2pt}
  {\fontsize{24}{27}\selectfont\titlespace}
  [\vspace{4pt}{\color{line}\titlerule[0.8pt]}]
\titleformat{name=\chapter,numberless}[display]
  {\xheavy\color{ink}}{}{0pt}{\fontsize{24}{27}\selectfont\titlespace}[\vspace{4pt}{\color{line}\titlerule[0.8pt]}]
\titlespacing*{\chapter}{0pt}{-10pt}{14pt}
\titleformat{\section}{\xheavy\Large\color{ink}}{\textcolor{accent}{\thesection}}{0.55em}{}
\titleformat{\subsection}{\xheavy\large\color{ink}}{\textcolor{accent}{\thesubsection}}{0.5em}{}
\titleformat{\subsubsection}{\bfseries\color{ink}}{}{0em}{}
\titlespacing*{\section}{0pt}{1.4em}{0.5em}
\titlespacing*{\subsection}{0pt}{1.1em}{0.35em}
\titlespacing*{\subsubsection}{0pt}{0.9em}{0.2em}
\setcounter{secnumdepth}{2}
\setcounter{tocdepth}{1}

% the table of contents in the same voice
\RequirePackage{tocloft}
\renewcommand{\cfttoctitlefont}{\xheavy\fontsize{24}{27}\selectfont\color{ink}}
\renewcommand{\cftchapfont}{\xmedium\color{ink}}
\renewcommand{\cftchappagefont}{\xmedium\color{accent}}
\renewcommand{\cftsecfont}{\color{ink}}
\renewcommand{\cftsecpagefont}{\color{muted}}
\renewcommand{\cftchapleader}{\cftdotfill{\cftdotsep}}
\setlength{\cftbeforechapskip}{0.5em}

% ----------------------------------------------------------------------------------- metadata
\newcommand{\xdoc@title}{X30}
\newcommand{\xdoc@header}{X30}
\newcommand{\xdoc@subtitle}{}
\newcommand{\xdoc@version}{1.0}
\newcommand{\xdoc@date}{\today}
\newcommand{\xdoc@audience}{}
\newcommand{\xdoc@note}{}
\newcommand{\xdoc@url}{https://darmawanrobotics.github.io/x30_ros/}
\newcommand{\doctitle}[1]{\renewcommand{\xdoc@title}{#1}}
\newcommand{\docheader}[1]{\renewcommand{\xdoc@header}{#1}}
\newcommand{\docsubtitle}[1]{\renewcommand{\xdoc@subtitle}{#1}}
\newcommand{\docversion}[1]{\renewcommand{\xdoc@version}{#1}}
\newcommand{\docdate}[1]{\renewcommand{\xdoc@date}{#1}}
\newcommand{\docaudience}[1]{\renewcommand{\xdoc@audience}{#1}}
\newcommand{\docnote}[1]{\renewcommand{\xdoc@note}{#1}}
\newcommand{\docurl}[1]{\renewcommand{\xdoc@url}{#1}}
\newcommand{\xdoc@revisions}{}
% \revision{1.0}{2026-11-21}{First version}
\newcommand{\revision}[3]{\g@addto@macro\xdoc@revisions{#1 & #2 & #3 \\}}

% ----------------------------------------------------------------------------------- headers
\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{\small\color{muted}\xdoc@header}
\fancyhead[R]{\small\color{muted}\nouppercase{\leftmark}}
\fancyfoot[C]{\small\xmedium\color{muted}\thepage\,/\,\pageref*{LastPage}}
\renewcommand{\headrule}{{\color{line}\hrule height 0.4pt}}
\renewcommand{\chaptermark}[1]{\markboth{\ifnum\value{chapter}>0 \thechapter.\ \fi #1}{}}
\fancypagestyle{plain}{\fancyhf{}%
  \fancyfoot[C]{\small\xmedium\color{muted}\thepage\,/\,\pageref*{LastPage}}%
  \renewcommand{\headrule}{}}

% ----------------------------------------------------------------------------------- cover
% Night-blue page, isotherm-like lines, the logo, the brand in gold, the title, the version,
% who the document is for and a QR code to the documentation site.
\newcommand{\makecover}{%
  \begin{titlepage}
  \thispagestyle{empty}
  \begin{tikzpicture}[remember picture,overlay]
    \fill[night] (current page.south west) rectangle (current page.north east);
    \begin{scope}[shift={(current page.south east)},opacity=0.16,line width=0.7pt,gold]
      \foreach \r in {2.2,3.4,...,15.4} {\draw (-1.2,2.4) circle (\r cm);}
    \end{scope}
    \fill[orange] ($(current page.north west)+(0,-6mm)$) rectangle
      ($(current page.north west)+(9mm,-62mm)$);
    \node[anchor=north west,inner sep=0] (logo) at ($(current page.north west)+(20mm,-22mm)$)
      {\includegraphics[width=\ifxdoc@afive 20mm\else 28mm\fi]{logo.pdf}};
    \node[anchor=north west,inner sep=0,text width=\paperwidth-40mm,align=left]
      (brand) at ($(logo.south west)+(0,-9mm)$)
      {\xheavy\color{gold}\addfontfeature{LetterSpace=28}\large EZRA\hspace{0.7em}ROBOTICS};
    \node[anchor=north west,inner sep=0,text width=\paperwidth-40mm,align=left]
      (title) at ($(brand.south west)+(0,-6mm)$)
      {\xheavy\color{white}%
       \ifxdoc@afive\fontsize{27}{29}\else\fontsize{38}{40}\fi\selectfont\titlespace \xdoc@title};
    \node[anchor=north west,inner sep=0,text width=\paperwidth-46mm,align=left]
      (sub) at ($(title.south west)+(0,-5mm)$)
      {\color{coversub}\ifxdoc@afive\large\else\Large\fi \xdoc@subtitle};
    \node[anchor=north west,inner sep=0] (ver) at ($(sub.south west)+(0,-9mm)$)
      {\xmedium\color{orange}\large Version \xdoc@version\quad\textbullet\quad \xdoc@date};
    \draw[nightsoft,line width=0.6pt] ($(current page.south west)+(20mm,40mm)$) --
      ($(current page.south east)+(-20mm,40mm)$);
    \node[anchor=north west,inner sep=0,text width=\paperwidth-80mm,align=left]
      at ($(current page.south west)+(20mm,36mm)$)
      {\color{coverfoot}\small \xdoc@audience};
    \node[anchor=north east,inner sep=2.2mm,fill=white,rounded corners=1.5mm]
      at ($(current page.south east)+(-20mm,36mm)$)
      {\qrcode[height=\ifxdoc@afive 14mm\else 19mm\fi]{\xdoc@url}};
  \end{tikzpicture}
  \end{titlepage}%
  \setcounter{page}{2}}

% "About this document": who it is for, where the rest is, what changed
\newcommand{\makeabout}{%
  \chapter*{About this document}
  \markboth{About this document}{}
  \addcontentsline{toc}{chapter}{About this document}
  \ifx\xdoc@audience\@empty\else
  \begin{tcolorbox}[enhanced,colback=accentsoft,colframe=accentsoft,arc=2pt,boxrule=0pt,
    left=8pt,right=8pt,top=6pt,bottom=6pt]
    {\xheavy\color{accent}For whom.}\enspace\xdoc@audience
  \end{tcolorbox}
  \fi
  \xdoc@note

  \section*{The X30 documents}
  \begin{tabularx}{\linewidth}{@{}>{\bfseries}p{0.30\linewidth}X@{}}
    \toprule
    Installation Guide & Getting the stack, the handheld app and NETRA onto a robot and a
      computer, from nothing to the first patrol. \\
    User Manual & Operating the robot: safety, the Skydroid H12, routes, patrols, inspections,
      the recorder and NETRA. \\
    Tuning Guide & What every setting does, the models behind them and how to tune a robot for a
      site; the full parameter reference. \\
    Architecture & How the stack is built: hosts, nodes, topics, frames, the safety pipeline,
      the protocols to the handheld and to NETRA. \\
    Site Acceptance Test & The handover tests, with pass criteria and sign-off. \\
    \bottomrule
  \end{tabularx}

  All of them, and the same pages as a website with search:
  \href{\xdoc@url}{\nolinkurl{\xdoc@url}}.

  \ifx\xdoc@revisions\@empty\else
  \section*{Revisions}
  \begin{tabularx}{\linewidth}{@{}p{0.11\linewidth}p{0.17\linewidth}X@{}}
    \toprule
    \bfseries Version & \bfseries Date & \bfseries Changes \\
    \midrule
    \xdoc@revisions
    \bottomrule
  \end{tabularx}
  \fi
  \cleardoublepage}

\newcommand{\makefront}{\makecover\makeabout\tableofcontents\clearpage}

% ----------------------------------------------------------------------------------- boxes
% \begin{notebox}[Title] ... \end{notebox}: notebox (blue), tipbox (green), warnbox (orange),
% dangerbox (red); the title defaults to Note, Tip, Warning, Danger. Kept whole on one page: a box
% split over two pages leaves a line or two alone.
\newcommand{\xdoc@box}[4]{%
  \newtcolorbox{#1}[1][#4]{enhanced,colback=#3,colframe=#2,boxrule=0pt,
    leftrule=3.2pt,arc=1.5pt,outer arc=1.5pt,left=7pt,right=7pt,top=5pt,bottom=5pt,
    before skip=8pt,after skip=8pt,
    fontupper=\color{ink},halign upper=flush left,
    title={##1},fonttitle=\xheavy\color{#2},coltitle=#2,
    attach title to upper={\enspace}}}
\xdoc@box{notebox}{accent}{accentsoft}{Note}
\xdoc@box{tipbox}{green}{greensoft}{Tip}
\xdoc@box{warnbox}{orange}{orangesoft}{Warning}
\xdoc@box{dangerbox}{red}{redsoft}{Danger}

% ----------------------------------------------------------------------------------- code
\lstdefinelanguage{yaml}{keywords={true,false,null},sensitive=false,comment=[l]{\#},
  morestring=[b]', morestring=[b]"}
\lstset{basicstyle=\ttfamily\small,columns=fullflexible,keepspaces=true,breaklines=true,
  breakatwhitespace=false,showstringspaces=false,upquote=true,
  backgroundcolor=\color{codeback},frame=l,framerule=2pt,rulecolor=\color{accent},
  xleftmargin=6pt,framesep=6pt,aboveskip=7pt,belowskip=7pt,
  commentstyle=\color{muted}\itshape,keywordstyle={},stringstyle={},
  literate={›}{{\textrangle}}1 {°}{{\textdegree}}1 {…}{{\ldots}}1 {·}{{\textperiodcentered}}1}

% ----------------------------------------------------------------------------------- words
% names in the stack: break at / _ . - and keep underscores
\makeatletter
\g@addto@macro\UrlBreaks{\do\_\do\.\do\-\do\/}
\makeatother
\newcommand{\code}[1]{\nolinkurl{#1}}
\newcommand{\pkg}[1]{\code{#1}}        % x30_navigation
\newcommand{\node}[1]{\code{#1}}       % path_follower_node
\newcommand{\topic}[1]{\code{#1}}      % /navigation/status
\newcommand{\srv}[1]{\code{#1}}        % navigation/start
\newcommand{\param}[1]{\code{#1}}      % max_vx
\newcommand{\file}[1]{\code{#1}}       % /etc/x30/navigation.env
\newcommand{\cmd}[1]{\texttt{\detokenize{#1}}}      % a short command inline: raw _ { } ~ (balanced braces)
\newcommand{\ui}[1]{\textbf{#1}}       % a label on a screen
\newcommand{\key}[1]{\tcbox[on line,colback=white,colframe=muted,boxrule=0.5pt,arc=2pt,
  left=2pt,right=2pt,top=0pt,bottom=0pt,boxsep=1pt]{\ttfamily\bfseries\small #1}}
\newcommand{\menupath}[1]{\textbf{#1}}  % Substation › Inspection points

% math
\newcommand{\R}{\mathbb{R}}
\newcommand{\vect}[1]{\boldsymbol{#1}}
\newcommand{\mat}[1]{\mathbf{#1}}
\newcommand{\T}[2]{{}^{#1}\mathbf{T}_{#2}}

% the parameter tables of tools/params.py
% an empty tick box for checklists and test sheets
\newcommand{\checkbox}{\tikz[baseline=-0.15ex]\draw[muted,line width=0.6pt,rounded corners=0.6pt] (0,-0.05em) rectangle (0.85em,0.8em);}
\newcommand{\readonlynote}{Parameters marked $^\dagger$ are read at start only
  (\code{live=False}); every other one can be changed while the node runs
  (\cmd{rosparam set}) and is checked before it applies.}
