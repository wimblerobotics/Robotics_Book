#! /usr/bin/bash
cd src/Robotics_Book/book/chapters
pandoc metadata.yaml \
brief_intro_to_bt.md \
creating_a_custom_behavior.md \
-o book.pdf --pdf-engine=xelatex --pdf-engine-opt=-shell-escape --highlight-style zenburn
cd ../../../../