#!/bin/bash
cat - | gvpack -u > graph.dot
dot -Tpdf -o out.pdf graph.dot
evince out.pdf
