#!/usr/bin/bash
cd book/chapters
pandoc metadata.yaml \
launch_files.md \
-o launch_files_chapter.pdf --pdf-engine=xelatex --pdf-engine-opt=-shell-escape --highlight-style kate \
--include-in-header=code-style.tex \
-V colorlinks -V linkcolor=NavyBlue -V urlcolor=NavyBlue -V citecolor=NavyBlue
cd ../../