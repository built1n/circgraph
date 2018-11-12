# circgraph

This is a tool for calculating equivalent
resistances/impedances/capacitances for a circuit (represented as a
graph, hence "circuit graph"), written in C++.

# Motivation

This program exists for three reasons: boredom, a desire to brush up
on my C++, and -- most importantly -- laziness. Why do tedious
arithmetic when you can spend hours writing a program to do it for
you?

# Usage

Compile with:

```
g++ main.cpp -o circgraph
```

`circgraph` takes input from stdin in the following format:

<source_node> <sink_node>
<node1> [<neighbor1> <resistance2>]...
<node2>
...
<noden>

See the `testX.txt` files for more.
