#! /usr/bin/bash
cd src/Robotics_Book/book/chapters
pandoc metadata.yaml \
setup.md \
what_is_ros.md \
ros_main_components_and_concepts.md \
a_bit_about_packages_and_nodes.md \
creating_your_first_workspace_and_package.md \
calibrating_odom.md \
-o book.pdf --pdf-engine=xelatex --pdf-engine-opt=-shell-escape --highlight-style zenburn
cd ../../../../