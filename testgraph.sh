#!/bin/bash
cat - | gvpack -u > graph.dot
dot -Goverlap=scale -Tpdf -o out.pdf graph.dot
evince out.pdf
