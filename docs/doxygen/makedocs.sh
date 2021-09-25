#!/bin/bash

doxygen DOXY_CFG 2> warnings.txt
cd latex
make pdf
cp refman.pdf ../doc_ros_diff_drive.pdf
