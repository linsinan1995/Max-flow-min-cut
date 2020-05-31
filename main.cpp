/*
* @Author: Lin Sinan
* @Github: https://github.com/linsinan1995
* @Email: mynameisxiaou@gmail.com
 * @LastEditors: Lin Sinan
* @Description: 
*               
*               
*               
*/
#include "include/graph.h"

using namespace mcmf;
using std::cout;

void maximum_flow(std::string &path)
{
    Graph graph = Graph().build(path);
    int maximum_flow = graph.maximum_flow();
    std::cout << "maximum flow: " << maximum_flow << "\n";
#ifdef GRAPHVIZ
    graph.toGraphViz("log.out");
#endif
}

void minimum_cut(std::string &path)
{
    Graph graph = Graph().build(path);
    graph.minimum_cut();
#ifdef GRAPHVIZ
    graph.toGraphViz("log.out");
#endif
}


void max_flow_min_cost(std::string &path)
{
    Graph graph = Graph().build(path);
    graph.min_cost_flow();
#ifdef GRAPHVIZ
    graph.toGraphViz("log.out");
#endif
}

int main()
{
    // if node is from 1 to nVertices => run cat data.txt | python preprocess.py > newdata.txt
    // to preprocess it first
    std::string filename {"data/data.in"};
    max_flow_min_cost(filename);
}