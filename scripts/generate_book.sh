#! /usr/bin/bash
cd src/Robotics_Book/book/chapters
pandoc metadata.yaml *.md -o book.pdf --pdf-engine=xelatex --pdf-engine-opt=-shell-escape -V colorlinks
cd ../../../../