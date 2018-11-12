/*
 * circgraph: a program for simplifying circuits
 *
 * Copyright (C) 2018 Franklin Wei
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 */

#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

struct edge {
    int id; /* connected node */
    double weight;
};

struct node {
    string name;
    vector<edge> neighbors;
};

vector<edge>::iterator find_edge(node &n, int id)
{
    for(vector<edge>::iterator it = n.neighbors.begin(); it != n.neighbors.end(); it++)
        if(it->id == id)
            return it;
    return n.neighbors.end();
}

bool is_doubly_connected(map<int, node> &graph, int a, int b)
{
    return find_edge(graph[a], b) != graph[a].neighbors.end() &&
        find_edge(graph[b], a) != graph[b].neighbors.end();
}

int count_edges(node &n, int id)
{
    int count = 0;
    for(vector<edge>::iterator it = n.neighbors.begin(); it != n.neighbors.end(); it++)
        if(it->id == id)
            count++;
    return count;
}

bool is_one_double_edge(map<int, node> graph, int s, int t)
{
    return is_doubly_connected(graph, s, t) &&
        count_edges(graph[s], t) == 1;
}

bool is_ynode(map<int, node> &graph, int id)
{
    node &n = graph[id];
    return n.neighbors.size() == 3 &&
        n.neighbors[0].id != n.neighbors[1].id &&
        n.neighbors[1].id != n.neighbors[2].id &&
        n.neighbors[0].id != n.neighbors[2].id &&
        is_doubly_connected(graph, id, n.neighbors[0].id) &&
        is_doubly_connected(graph, id, n.neighbors[1].id) &&
        is_doubly_connected(graph, id, n.neighbors[2].id);
}

double combine_s(double a, double b)
{
    /* change for capacitance */
    return a + b;
}

double combine_p(double a, double b)
{
    return 1 / ( 1 / a + 1 / b );
}

double combine_ydelta(double adj1, double adj2, double opp)
{
    double p = adj1 * adj2 + adj1 * opp + adj2 * opp;
    return p / opp;
}

double combine_deltay(double adj1, double adj2, double opp)
{
    return adj1 * adj2 / ( adj1 + adj2 + opp );
}

void dump_graph(map<int, node> graph)
{
    for(map<int, node>::iterator it = graph.begin(); it != graph.end(); it++)
    {
        cerr << "Node " << it->first << ": ";
        struct node &e = it->second;
        for(vector<edge>::iterator j = e.neighbors.begin(); j != e.neighbors.end(); j++)
            cerr << j->id << " ";
        cerr << endl;
    }
}

void dump_dot(map<int, node> graph)
{
    cout << "graph a {" << endl;
    for(map<int, node>::iterator it = graph.begin(); it != graph.end(); it++)
    {
        //cerr << "Node " << it->first << ": ";
        struct node &e = it->second;
        for(vector<edge>::iterator j = e.neighbors.begin(); j != e.neighbors.end(); j++)
        {
            //cout << graph[it->first].name << " -> " <<  graph[j->id].name << " [label=\"" << j->weight << "\"];" << endl;
            if(is_doubly_connected(graph, it->first, j->id))
                cout << it->first << " -- " << j->id << " [label=\"" << j->weight << "\"];" << endl;
        }
        //cerr << endl;
    }
    cout << "}" << endl;
}

void insert_edge(map<int, node> &graph, int id_a, int id_b, double weight)
{
    //cerr << "Inserting edge " << id_a << "-" << id_b << endl;
    node &a = graph[id_a], &b = graph[id_b];

    edge ab, ba;
    ab.id = id_b;
    ab.weight = weight;

    ba.id = id_a;
    ba.weight = weight;

    a.neighbors.push_back(ab);
    b.neighbors.push_back(ba);
}

node &insert_node(map<int, node> &graph, int id)
{
    node new_node;
    map<int, node>::iterator it = graph.insert(pair<int, node>(id, new_node)).first;
    return it->second;
}

node &insert_node_if_new(map<int, node> &graph, int id)
{
    map<int, node>::iterator it = graph.find(id);
    if(it == graph.end())
        return insert_node(graph, id);
    else
        return it->second;
}

void erase_edges(map<int, node> &graph, int id_a, int id_b)
{
    //cerr << "Erasing edges " << id_a << "-" << id_b << endl;
    node &a = graph[id_a], &b = graph[id_b];

    vector<edge>::iterator it;

    while((it = find_edge(a, id_b)) != a.neighbors.end())
        a.neighbors.erase(it);

    while((it = find_edge(b, id_a)) != b.neighbors.end())
        b.neighbors.erase(it);
}

void do_ydelta(map<int, node> &graph, int node_id)
{
    /* assumes ynode */
    node &n = graph[node_id];

    double w1 = n.neighbors[0].weight,
        w2 = n.neighbors[1].weight,
        w3 = n.neighbors[2].weight;

    double wa = combine_ydelta(w2, w3, w1);
    double wb = combine_ydelta(w1, w3, w2);
    double wc = combine_ydelta(w1, w2, w3);

    /* add delta edges */
    insert_edge(graph, n.neighbors[1].id, n.neighbors[2].id, wa);
    insert_edge(graph, n.neighbors[0].id, n.neighbors[2].id, wb);
    insert_edge(graph, n.neighbors[0].id, n.neighbors[1].id, wc);

    /* delete Y edges */
    for(int i = 0; i < 3; ++i)
    {
        node &neighbor = graph[n.neighbors[i].id];
        neighbor.neighbors.erase(find_edge(neighbor, node_id));
    }

    n.neighbors.clear();
}

void do_deltay(map<int, node> &graph, int id_a, int id_b, int id_c)
{
    node &a = graph[id_a], &b = graph[id_b];

    double wa = find_edge(b, id_c)->weight;
    double wb = find_edge(a, id_c)->weight;
    double wc = find_edge(a, id_b)->weight;

    double w1 = combine_deltay(wb, wc, wa);
    double w2 = combine_deltay(wa, wc, wb);
    double w3 = combine_deltay(wa, wb, wc);

    int id_d = graph.rbegin()->first + 1;

    insert_node(graph, id_d);
    //graph[id_d].name = "Ynode";

    insert_edge(graph, id_a, id_d, w1);
    insert_edge(graph, id_b, id_d, w2);
    insert_edge(graph, id_c, id_d, w3);

    erase_edges(graph, id_a, id_b);
    erase_edges(graph, id_a, id_c);
    erase_edges(graph, id_b, id_c);
}

bool do_p_transforms(map<int, node> &graph, int node_id, int other_node = -1)
{
    node &a = graph[node_id];

    bool progress = false;

    /* perform P transform: O(n^2)? */
    for(vector<edge>::iterator i = a.neighbors.begin(); i < a.neighbors.end(); i++)
    {
        for(vector<edge>::iterator j = a.neighbors.begin(); j < a.neighbors.end(); j++)
        {
            if(i == j)
                continue;

            if(i->id == j->id &&
               (other_node < 0 || other_node == i->id))
            {
                cerr << "Performing P-transform on duplicate " << node_id << "-" << i->id << " edges." << endl;

                /* remove edge k */
                double new_weight = combine_p(i->weight, j->weight);

                i->weight = new_weight;

                /* will be incremented */
                j = a.neighbors.erase(j) - 1;

                /* recurse to handle the opposite direction */
                do_p_transforms(graph, i->id, node_id);

                if(other_node < 0)
                    dump_dot(graph);

                progress = true;
            }
        }
    }

    return progress;
}

/* ydelta and deltay control whether the Y->delta and delta->Y
 * transforms are performed, respectively */
pair<bool, int> simp_iter(map<int,node> &graph, int s, int t, bool ydelta, bool deltay)
{
    bool progress = false;

    /* breadth-first search */
    set<int> visited;
    set<int> open;
    open.insert(s);

    int t_visited = 0;

    while(open.size())
    {
        for(set<int>::iterator i = open.begin(); i != open.end();)
        {
            int id_a = *i;
            node &a = graph[id_a];

            cerr << "Visiting node " << id_a << endl;

            if(id_a == t)
                t_visited++;

            vector<edge> &neighbors = a.neighbors;

            /* First remove duplicate edges with the P transform */
            progress |= do_p_transforms(graph, id_a);

            /* Loop over neighbors */
            for(vector<edge>::iterator j = neighbors.begin(); j < neighbors.end(); j++)
            {
                edge &ed_ab = *j;
                int id_b = ed_ab.id;

                /* get the connected node */
                node &b = graph[ed_ab.id];

                cerr << "Connected node " << ed_ab.id << " has " << b.neighbors.size() << " emanating edges." << endl;

                /* perform S transform */
                if(id_b != s && id_b != t &&
                   b.neighbors.size() == 2 &&
                   b.neighbors[0].id != b.neighbors[1].id)
                {
                    /* Replace A-B-C with A-C */
                    edge &ed_bc = b.neighbors[0].id == id_a ? b.neighbors[1] : b.neighbors[0];
                    int id_c = ed_bc.id;

                    if(is_doubly_connected(graph, id_b, id_c))
                    {
                        cerr << "Performing S-transform on edges " << id_a << "-" << id_b << "-" << id_c << endl;

                        /* note that we have to do the replacement on both
                         * nodes, because edges are directed */
                        double new_weight = combine_s(ed_ab.weight, ed_bc.weight);

                        ed_ab.id = id_c;
                        ed_ab.weight = new_weight;

                        node &c = graph[ed_bc.id];

                        /* find the edge that goes to node b (with id_b) */
                        edge &ed_cb = *find_edge(c, id_b);

                        ed_cb.id = id_a;
                        ed_cb.weight = new_weight;

                        /* node B becomes an orphan node */
                        dump_dot(graph);

                        progress = true;
                    }
                }

                /* Don't mark if already visited */
                if(visited.find(ed_ab.id) != visited.end())
                    continue;

                /* Mark neighbors for next iteration. We don't check
                 * for the node already being in the open set, because
                 * it is a set. */
                open.insert(ed_ab.id);
            }

            if(ydelta)
            {
                /* Try the Y->delta transform */
                if(id_a != s && id_a != t && is_ynode(graph, id_a))
                {
                    cerr << "Performing Y-delta transform on node " << id_a << endl;

                    do_ydelta(graph, id_a);

                    dump_dot(graph);
                    progress = true;
                }
            }

            if(deltay)
            {
                /* Try the delta-Y transform (this is the inverse of
                 * Y-delta, so we must be careful about enabling
                 * it). */

                /* Look for any pair of nodes that are directly
                 * connected (only one edge between them). We don't
                 * have to worry about multiple adjacent edges because
                 * P-transforms are done earlier. */
                for(int i = 0; i < a.neighbors.size(); i++)
                {
                    for(int j = 0; j < a.neighbors.size(); j++)
                    {
                        if(i == j)
                            continue;

                        int id_b = a.neighbors[i].id;
                        int id_c = a.neighbors[j].id;

                        node &b = graph[id_b];
                        if(is_doubly_connected(graph, id_b, id_c) &&
                           count_edges(b, id_c) == 1)
                        {
                            cerr << "Performing delta-Y transform on " << id_a << ", " << id_b << ", " << id_c << endl;

                            do_deltay(graph, id_a, id_b, id_c);

                            dump_dot(graph);
                            progress = true;
                        }
                    }
                }
            }

            visited.insert(id_a);

            /* hack */
            set<int>::iterator next = ++i;
            open.erase(--i);
            i = next;
        }
    }

    return pair<bool, int>(progress, t_visited);
}

/* source, sink */
double simp_graph(map<int,node> &graph, int s, int t)
{
    int t_visited = 0;

    /* Need to refine this termination condition. There must be
     * exactly one path between s and t... the current condition isn't
     * sufficient. */

    while(!(is_one_double_edge(graph, s,t) && t_visited == 1))
    {
        /* iterate over nodes in a breadth-first fashion */
        pair<bool, int> ret = simp_iter(graph, s, t, true, false);
        t_visited = ret.second;
        if(!ret.first)
        {
            cerr << "Making no progress, saw t " << t_visited << " times." << endl;
            return 0;
        }
    }

    return graph[s].neighbors[0].weight;
}

int get_node_id(map<string, int> &names, string name, int &counter)
{
    const map<string, int>::const_iterator it = names.find(name);
    if(it == names.end())
        return names[name] = counter++;
    else
        return it->second;
}

int main()
{
    /* construct graph from adjacency list */
    map<int, node> graph;
    string line;
    map<string, int> names;
    string s, t;

    getline(cin, line);
    stringstream ss(line);
    ss >> s >> t;

    cerr << "Source and sink are " << s << ", " << t << endl;

    int id_counter = 1;

    while(getline(cin, line))
    {
        ss = stringstream(line);

        double weight;
        string a, b;
        ss >> a >> b >> weight;

        int id_a, id_b;
        id_a = get_node_id(names, a, id_counter);
        id_b = get_node_id(names, b, id_counter);

        if(id_a == id_b)
            cerr << "WARNING: Self-loop!" << endl;

        insert_node_if_new(graph, id_a);
        insert_node_if_new(graph, id_b);

        graph[id_a].name = a;
        graph[id_b].name = b;

        insert_edge(graph, id_a, id_b, weight);
    }

    int id_s, id_t;
    id_s = names.at(s);
    id_t = names.at(t);

    dump_dot(graph);

    double eq = simp_graph(graph, id_s, id_t);

    cerr << "Equivalent resistance: " << eq << endl;
}
