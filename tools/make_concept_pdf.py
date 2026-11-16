#!/usr/bin/env python3
"""Typeset docs/CONCEPT_GUIDE.md as a PDF, with every figure embedded in the
graph guide (Part G) next to its reading instructions.

    python3 tools/make_concept_pdf.py      # -> docs/concept_guide/concept_guide.pdf

Needs pandoc and xelatex (TeX Live).
"""
import os
import re
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "docs", "CONCEPT_GUIDE.md")
OUT = os.path.join(ROOT, "docs", "concept_guide")
SEARCH = [os.path.join(ROOT, "docs", "figures"), os.path.join(ROOT, "results", "figures_ext"),
          os.path.join(ROOT, "results", "figures_signals")]
PREFER = {"xcheck_trajectories.png": "xcheck_trajectories_fixedgate.png",
          "cross_stack_scatter.png": "cross_stack_scatter_fixedgate.png",
          "e1_both_stacks.png": "e1_both_stacks_fixedgate.png",
          "e2_noise_both_stacks.png": "e2_noise_both_stacks_fixedgate.png"}


def find(name):
    name = os.path.basename(name)
    for cand in (PREFER.get(name, name), name):
        for d in SEARCH:
            p = os.path.join(d, cand)
            if os.path.exists(p):
                return p
    return None


def preprocess(md):
    out, in_g = [], False
    lines = md.splitlines()
    for ln in lines:
        if ln.startswith("## Part G"):
            in_g = True
        elif ln.startswith("## Part H"):
            in_g = False
        out.append(ln)
        m = re.match(r"### (G\d+)\. (.*)", ln)
        if in_g and m:
            names = re.findall(r"`([^`]+\.png)`", ln)
            seen = []
            for n in names:
                p = find(n)
                if p and p not in seen:
                    seen.append(p)
            for p in seen[:3]:
                out += ["", f"![]({p}){{width=95%}}", ""]
    md = "\n".join(out)
    # links to other repository documents: keep the text, drop the target
    md = re.sub(r"\[([^\]]+)\]\((?!#|http)[^)]+\.md(#[^)]*)?\)", r"\1", md)
    return md


HEADER = r"""
\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{\small Cooperative payload transport --- concept guide}
\fancyhead[R]{\small\thepage}
\renewcommand{\headrulewidth}{0.3pt}
\usepackage{xcolor}
\definecolor{acc}{HTML}{2A78D6}
\usepackage{sectsty}
\allsectionsfont{\color{acc}\sffamily}
\setlength{\parskip}{4pt}
\usepackage{float}
\let\origfigure\figure
\let\endorigfigure\endfigure
\renewenvironment{figure}[1][2] {\expandafter\origfigure\expandafter[H]} {\endorigfigure}
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    md = preprocess(open(SRC, encoding="utf-8").read())
    md = md.replace("# Concept guide: learn the project, read every graph", "", 1)
    mdp = os.path.join(OUT, "concept_guide_build.md")
    open(mdp, "w", encoding="utf-8").write(md)
    hp = os.path.join(OUT, "header.tex")
    open(hp, "w").write(HEADER)
    tex = os.path.join(OUT, "concept_guide.tex")
    meta = ["-M", "title=Cooperative Payload Transport by a Robot Formation",
            "-M", "subtitle=Concept guide: learn every idea, read every graph",
            "-M", "author=Companion to the benchmark and the PA-VFH manuscript",
            "-M", "date=November 2026"]
    subprocess.run(["pandoc", mdp, "-s", "-o", tex, "--toc", "--toc-depth=2",
                    "--shift-heading-level-by=-1", "-V", "documentclass=article", "-V", "papersize=a4",
                    "-V", "geometry:margin=2.2cm", "-V", "fontsize=10.5pt",
                    "-V", "mainfont=DejaVu Serif", "-V", "sansfont=DejaVu Sans",
                    "-V", "monofont=DejaVu Sans Mono", "-V", "linkcolor=acc", "-V", "colorlinks=true",
                    "-V", "urlcolor=acc", "-V", "toccolor=black",
                    "-H", hp, "--pdf-engine=xelatex"] + meta, check=True)
    for _ in range(2):
        r = subprocess.run(["xelatex", "-interaction=nonstopmode", "concept_guide.tex"],
                           cwd=OUT, capture_output=True, text=True)
    pdf = os.path.join(OUT, "concept_guide.pdf")
    if not os.path.exists(pdf):
        print(r.stdout[-3000:])
        raise SystemExit("xelatex failed")
    print("wrote", os.path.relpath(pdf, ROOT))


if __name__ == "__main__":
    main()
