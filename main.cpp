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
    vector<edge> neighbors;
};

bool is_done(map<int, node> graph, int s, int t)
{
    return graph[s].neighbors.size() == 1 &&
        graph[s].neighbors[0].id == t &&
        graph[t].neighbors.size() == 1 &&
        graph[t].neighbors[0].id == s;
}

bool is_ynode(node &n)
{
    return n.neighbors.size() == 3 &&
        n.neighbors[0].id != n.neighbors[1].id &&
        n.neighbors[1].id != n.neighbors[2].id &&
        n.neighbors[0].id != n.neighbors[2].id;
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
    cout << "digraph a {" << endl;
    for(map<int, node>::iterator it = graph.begin(); it != graph.end(); it++)
    {
        //cerr << "Node " << it->first << ": ";
        struct node &e = it->second;
        for(vector<edge>::iterator j = e.neighbors.begin(); j != e.neighbors.end(); j++)
        {
            cout << it->first << " -> " <<  j->id << " [label=\"" << j->weight << "\"];" << endl;
        }
        //cerr << endl;
    }
    cout << "}" << endl;
}

vector<edge>::iterator find_edge(node &n, int id)
{
    for(vector<edge>::iterator it = n.neighbors.begin(); it != n.neighbors.end(); it++)
        if(it->id == id)
            return it;
    return n.neighbors.end();
}

double combine_ydelta(double w1, double w2, double w3, double opp)
{
    double p = w1 * w2 + w1 * w3 + w2 * w3;
    return p / opp;
}

void insert_edge(map<int, node> &graph, int id_a, int id_b, double weight)
{
    cerr << "Inserting edge " << id_a << "-" << id_b << endl;
    node &a = graph[id_a], &b = graph[id_b];

    edge ab, ba;
    ab.id = id_b;
    ab.weight = weight;

    ba.id = id_a;
    ba.weight = weight;

    a.neighbors.push_back(ab);
    b.neighbors.push_back(ba);
}

void do_ydelta(map<int, node> &graph, int node_id)
{
    /* assumes ynode */
    node &n = graph[node_id];

    double w1 = n.neighbors[0].weight,
        w2 = n.neighbors[1].weight,
        w3 = n.neighbors[2].weight;

    double wa = combine_ydelta(w1, w2, w3, w1);
    double wb = combine_ydelta(w1, w2, w3, w2);
    double wc = combine_ydelta(w1, w2, w3, w3);

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

void do_p_transforms(map<int, node> &graph, int node_id, int other_node = -1)
{
    node &a = graph[node_id];

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

                dump_dot(graph);
            }
        }
    }
}

void simp_iter(map<int,node> &graph, int s, int t, bool ydelta)
{
    /* breadth-first search */
    set<int> visited;
    set<int> open;
    open.insert(s);

    while(open.size())
    {
        for(set<int>::iterator i = open.begin(); i != open.end();)
        {
            int id_a = *i;
            node &a = graph[id_a];

            cerr << "Visiting node " << id_a << endl;

            vector<edge> &neighbors = a.neighbors;

            /* First remove duplicate edges with the P transform */
            do_p_transforms(graph, id_a);

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
                if(is_ynode(a))
                {
                    cerr << "Performing Y-delta transform on node " << id_a << endl;

                    do_ydelta(graph, id_a);

                    dump_dot(graph);
                }
            }

            visited.insert(id_a);

            /* hack */
            set<int>::iterator next = ++i;
            open.erase(--i);
            i = next;
        }
    }
}

/* source, sink */
void simp_graph(map<int,node> &graph, int s, int t)
{
    dump_dot(graph);

    while(!is_done(graph, s, t))
    {
        /* iterate over nodes in a breadth-first fashion */
        simp_iter(graph, s, t, true);
    }
}

int main()
{
    /* construct graph from adjacency list */
    map<int, node> graph;
    string line;
    int s, t;
    cin >> s >> t;

    while(getline(cin, line))
    {
        stringstream ss(line);

        int node_id;
        struct node n;
        ss >> node_id;

        struct edge e;
        while(ss >> e.id && ss >> e.weight)
        {
            cerr << "Adding neighbor " << e.id << " with weight " << e.weight << endl;

            n.neighbors.push_back(e);
        }

        graph.insert(std::pair<int, node>(node_id, n));
    }

    simp_graph(graph, s, t);
}
