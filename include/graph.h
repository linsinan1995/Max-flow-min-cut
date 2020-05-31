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
#pragma once

#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>
#include <algorithm>
#include <queue>
#include <optional>
#include <functional>

namespace mcmf {

    struct FlowEdge
    {
        int from, to, capacity, flow, cost;
    };

    class Graph {
        // used to add log
        enum Action { addArc, addArcText, stop, highlightArc, addNodeText};
    public:
        const int INF = 0x3fffffff;
        // using FlowEdge = std::tuple<int, int, int, int>;
        using Edge = std::pair<int, int>;
        using ResidualGraph = std::vector<std::vector<int>>;
    public:
        Graph() = default;
        Graph& build(const std::string &path, const char &delimiter = ' ');
        void plot();
        std::vector<std::string> getLog();
        template <typename Comparator>
        std::optional<std::vector<int>> bfs(Comparator &);

        template <typename Comparator>
        std::vector<int> dfs(const int &node, std::vector<bool> &visited, const Comparator &cmp);
        template <typename Comparator>
        std::vector<int> dfs(const int &startNode, const Comparator &cmp);

        int maximum_flow();
        int maximum_flow(ResidualGraph &);
        std::vector<Edge> minimum_cut();
        std::vector<Edge> minimum_cut(const ResidualGraph &);
        std::optional<std::vector<int>> shortestPath(std::vector<FlowEdge> &residual_cost_graph,
                                                     std::vector<std::vector<int>> &edge_index);
        int min_cost_flow();

    #ifdef GRAPHVIZ
        void toGraphViz(const std::string &);
    #endif

    private:
        std::tuple<int, int, int, int> split(const std::string &line, const char &delimiter);
        void addEdge(const int &emanatingNode,
                     const int &terminatingNode,
                     const int &maxCapacity,
                     const int &cost);
    #ifdef GRAPHVIZ
        void addLog(Action action, std::vector<std::string> &&infos);
        void addLog(Action action, std::vector<int> &&infos);
        void logUpdate();
        std::vector<std::string> logs;
    #endif

    public:
        std::vector<std::vector<Edge>> adjacency_matrix;
        int sourceNode,sinkNode,nVertices,nArcs;
    };

    inline
    Graph& Graph::build(const std::string &path, const char &delimiter)
    {
        std::vector<std::string> lines {};
        std::ifstream in(path);
        if (!in.is_open()) {
            throw std::logic_error("Can't open the file.");
        }

        std::string line, token;
        std::getline(in, line);
        std::tie(nVertices, nArcs, sourceNode, sinkNode) = split(line, delimiter);

        std::vector<Edge> row (nVertices, std::make_pair(0, 0x7ffffff));
        adjacency_matrix = std::vector<std::vector<Edge>> (nVertices, row);

        while(std::getline(in, line)) {
            auto[emanatingNode, terminatingNode, maxCapacity, cost] = split(line, delimiter);
            addEdge(emanatingNode, terminatingNode, maxCapacity, cost);
        }
    #ifdef GRAPHVIZ
        addLog(Action::addNodeText, {std::to_string(sourceNode), "Source"});
        addLog(Action::addNodeText, {std::to_string(sinkNode), "Sink"});
        logUpdate();
    #endif
        return *this;
    }



    inline
    std::tuple<int, int, int, int> Graph::split(const std::string &line, const char &delimiter)
    {
        std::vector<int> tokens {};
        int pos_begin = 0, pos_end = 0;

        while(pos_end != -1){
            pos_end = line.find(delimiter, pos_begin);
            tokens.push_back(
                std::stoi(line.substr(pos_begin, line.find(delimiter, pos_begin) - pos_begin))
            );
            pos_begin = pos_end + 1;
        }

        if (tokens.size() != 4)
        {
            throw std::logic_error("data has a wrong format!\n");
        }


        return {tokens[0], tokens[1], tokens[2], tokens[3]};
    }


    inline
    void Graph::addEdge(const int &emanatingNode,
                        const int &terminatingNode,
                        const int &maxCapacity,
                        const int &cost)
    {
        adjacency_matrix[emanatingNode][terminatingNode] = std::make_pair(maxCapacity, cost);
    #ifdef GRAPHVIZ
        addLog(Action::addArc, {emanatingNode, terminatingNode});
        addLog(Action::addArcText, {std::to_string(emanatingNode),
                                 std::to_string(terminatingNode) ,
                                 "cap: " + std::to_string(maxCapacity) + " cost:" +  std::to_string(cost)});
    #endif
    }


    inline
    void Graph::plot() {
        for (const auto &row : adjacency_matrix) {
            for (const auto &item : row) {
                std::cout << "(" << item.first << ", " << item.second << ") ";
            }
            std::cout << "\n";
        }
    }

    #ifdef GRAPHVIZ
    void Graph::addLog(Graph::Action action, std::vector<int> &&infos) {
        std::vector<std::string> strInfos {};

        std::transform(infos.begin(),
                       infos.end(),
                       std::back_inserter(strInfos),
                       [](int &n) { return std::to_string(n); }
        );

        addLog(action, std::move(strInfos));
    }


    inline
    void Graph::addLog(Action action, std::vector<std::string> &&infos)
    {
        std::string log {};

        for (std::size_t idx = 0; idx < infos.size(); idx++)
        {
            std::string info = infos[idx];

            if (action == Action::addNodeText && idx == 1)
            {
                info += "\"" ;
                info.insert(0, "\"");
            }

            if (action == Action::addArcText && idx == 2)
            {
                info.insert(0, "\"");
                if (infos.size() == 3)
                {
                    info += "\"";
                }
            }

            if (action == Action::addArcText && idx == 3)
            {
                info +=  "\"";
            }

            log += " " + info;
        }

        switch (action)
        {
            case Action::addArc :
                logs.emplace_back("ae" + log + "\n");
                break;
            case Action::addArcText :
                logs.emplace_back("le" + log + "\n");
                break;
            case Action::highlightArc :
                logs.emplace_back("he" + log + "\n");
                break;
            case Action::stop :
                logs.emplace_back("ns\n");
                break;
            case Action::addNodeText :
                logs.emplace_back("ln" + log + "\n");
                break;
        };
    }


    void Graph::logUpdate() {
        logs.emplace_back("ns\n");
    }

    inline
    std::vector<std::string> Graph::getLog()
    {
        return logs;
    }

    void Graph::toGraphViz(const std::string &filename) {
        std::ofstream file(filename);
        for(auto &log : logs)
            file << log;

        file.close();
    }
    #endif

    template <typename Comparator> inline
    std::optional<std::vector<int>> Graph::bfs(Comparator &cmp)
    {
        // a traceback table, which stores the info about the trace of bfs
        std::vector<int> path (nVertices, -1);

        // it raises bug in old version gcc and clang because integer may be narrowly converted into bool
        // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65043#c5
        std::vector<bool> visited (nVertices, false);
    //    std::vector<bool> visited {};
    //    visited.reserve(nVertices);

        std::queue <int> q {{sourceNode}};
        visited[sourceNode] = true;

        while (!q.empty())
        {
            int node = q.front(); q.pop();
            for (int idx = 0; idx < nVertices; idx++)
            {
                if (!visited[idx] && cmp(node, idx)) // adjacency_matrix[q][idx].first > 0)
                {
                    q.push(idx);
                    path[idx] = node; //
                    visited[idx] = true;
                }
            }

        }
        return visited[sinkNode] ? std::optional<std::vector<int>> {path}
                                     : std::nullopt;
    }

    inline
    int Graph::maximum_flow()
    {
        ResidualGraph r {};
        return maximum_flow(r);
    }

    inline
    int Graph::maximum_flow(ResidualGraph &residual_graph)
    {
        int maxFlow = 0;


        // copy a residual graph from adjacency_matrix, and we only consider capacity.
        residual_graph.clear();
        for (const auto &row : adjacency_matrix)
        {
            std::vector<int> row_residual_graph {};
            for (const auto &[capacity, cost] : row)
            {
                row_residual_graph.push_back(capacity);
            }
            residual_graph.push_back(row_residual_graph);
        };

        // wrap residual_graph reference in a lambda function to pass variable
        auto cmp = [&residual_graph] (const int &node1, const int &node2) -> bool {
            return residual_graph[node1][node2] > 0;
        };

        int cnt = 1;
        while(auto optPath = bfs(cmp))
        {
            std::cout << "===========================\n";
            std::cout << "         Iter " << cnt++ << "     \n";
            std::vector<int> path = std::move(*optPath);
            int pathFlow = INF;

            for (int u = sinkNode; u != sourceNode; u = path[u])
            {
                int v = path[u];
                pathFlow = std::min(pathFlow, residual_graph[v][u]);
            }

            for (int u = sinkNode; u != sourceNode; u = path[u])
            {
                int v = path[u];
                residual_graph[v][u] -= pathFlow;
                residual_graph[u][v] += pathFlow;
                std::cout << u << " <- ";
    #ifdef GRAPHVIZ
                addLog(Action::highlightArc, {v, u});
                addLog(Action::addArcText, {std::to_string(v),
                                            std::to_string(u),
                                            "cap: " + std::to_string(residual_graph[v][u]) +
                                            " cost: " + std::to_string(adjacency_matrix[v][u].second)});
    #endif
            }
            maxFlow += pathFlow;

            std::cout << sourceNode << "\npath flow: " << pathFlow << "\ntotal flow: " << maxFlow << "\n";
            std::cout << "===========================\n\n";
    #ifdef GRAPHVIZ
            logUpdate();
    #endif
        }

        return maxFlow;
    }

    template <typename Comparator> inline
    std::vector<int> Graph::dfs(const int &node, std::vector<bool> &visited, const Comparator &cmp) {
        visited[node] = true;
        for (int next = 0; next < nVertices; next++)
            if (!visited[next] && cmp(node, next))
                dfs(next, visited, cmp);
        return {};
    }

    template<typename Comparator> inline
    std::vector<int> Graph::dfs(const int &node, const Comparator &cmp) {
        std::vector<bool> visited (nVertices, false);
        return dfs(node, visited, cmp);
    }


    inline
    std::vector<Graph::Edge> Graph::minimum_cut(const ResidualGraph &residual_graph) {
        std::vector<bool> visited(nVertices, false);

        // wrap residual_graph reference in a lambda function to pass variable
        auto cmp = [&residual_graph](const int &node1, const int &node2) -> bool {
            return residual_graph[node1][node2] > 0;
        };


        dfs(sourceNode, visited, cmp);
        std::vector<Edge> cuts{};
        for (int u = 0; u < nVertices; u++)
            for (int v = 0; v < nVertices; v++)
                if (visited[u] && !visited[v] && adjacency_matrix[u][v].first > 0)
                {
                    cuts.emplace_back(u, v);
    #ifdef GRAPHVIZ
                    addLog(Action::highlightArc, {u, v});
                }

        logUpdate();
    #else
                }
    #endif

        for (auto [src, sink] : cuts)
        {
            std::cout << "cut: " << src << "->" << sink << "\n";
        }
        return cuts;
    }

    inline
    std::vector<Graph::Edge> Graph::minimum_cut() {
        ResidualGraph residual_graph {};
        maximum_flow(residual_graph);
        return minimum_cut(residual_graph);
    }




    int Graph::min_cost_flow()
    {
        int maxFlow = 0, minCost = 0;
        // build a residual cost graph by adjaceny list, since there are multiple edges in an arc
        std::vector<std::vector<int>> edgesIndex (nVertices);
        std::vector<FlowEdge> residual_cost_graph(nVertices);


        for (int from = 0; from < nVertices; from++)
        {
            for (int to = 0; to < nVertices; to++)
                // check if the edge exists
                if (adjacency_matrix[from][to].first > 0)
                {
                    auto [capacity, cost] = adjacency_matrix[from][to];
                    residual_cost_graph.push_back({from,to,capacity,0,cost});
                    edgesIndex[from].push_back(residual_cost_graph.size()-1);
                    residual_cost_graph.push_back({to,from,0,0,-1*cost});
                    edgesIndex[to].push_back(residual_cost_graph.size()-1);
                }
        }

        int it = 1;
        while(auto optPath = shortestPath(residual_cost_graph, edgesIndex)) {
            int pathFlow = INF, pathCost = 0;
            std::vector<int> path = std::move(*optPath);
            for (int u = sinkNode; u != sourceNode; u = residual_cost_graph[path[u]].from) {
                int arc_index = path[u];
                pathCost += residual_cost_graph[arc_index].cost;
                pathFlow = std::min(pathFlow,
                                    residual_cost_graph[arc_index].capacity -
                                    residual_cost_graph[arc_index].flow);
            }

            for (int u = sinkNode; u != sourceNode; u = residual_cost_graph[path[u]].from) {
                int idx = path[u];
                residual_cost_graph[idx].flow += pathFlow;
                // idx ^ 1 => find the opposite arc
                residual_cost_graph[idx ^ 1].flow += pathFlow;
    #ifdef GRAPHVIZ
                auto [from, to, cap, flow, cost] = residual_cost_graph[idx];
                addLog(Action::highlightArc, {from, to});
                addLog(Action::addArcText, {std::to_string(from),
                                            std::to_string(to),
                                            "cap: " + std::to_string(cap - flow) +
                                            " cost: "+ std::to_string(cost)});
            }
            logUpdate();
    #else
            }
    #endif

            // print path
            std::cout << "===========================\n";
            std::cout << "         Iter " << it++ << "     \n";
            std::cout << "flow:" << pathFlow << "\ncost:" << pathCost << "\n";
            for (int u = sinkNode; u != sourceNode; u = residual_cost_graph[path[u]].from)
            {
                int idx = path[u];
                std::cout << residual_cost_graph[idx].to << "<-";
            }
            std::cout << sourceNode << "\n" << "===========================\n\n";

            maxFlow += pathFlow;
            minCost += pathFlow * pathCost;
        }
        std::cout << "maximum flow: " << maxFlow << "\n";
        std::cout << "total cost: " << minCost << "\n";
        return minCost;

    }



    std::optional<std::vector<int>> Graph::shortestPath(std::vector<FlowEdge> &residual_cost_graph,
                                                        std::vector<std::vector<int>> &edge_index)
    {
        // Bellman-Ford with queue
        std::vector<int> path (nVertices, 0), dist(nVertices, INF);
        std::vector<bool> inQueue(nVertices, false); // if node is in queue
        std::queue<int> q {{sourceNode}};

        dist[sourceNode] = 0;
        inQueue[sourceNode] = true;
        while(!q.empty())
        {
            int u = q.front(); q.pop();
            inQueue[u] = false;
            for (std::size_t i = 0; i < edge_index[u].size(); i++)
            {
                auto [from, to, capacity, flow, cost] = residual_cost_graph[edge_index[u][i]];

                if (capacity > flow && dist[to] > dist[u] + cost)
                {
                    path[to] = edge_index[u][i];
                    dist[to] = dist[u] + cost;
                    if (!inQueue[to])
                    {
                        q.push(to);
                        inQueue[to] = true;
                    }
                }

            }
        }

        return dist[sinkNode] != INF ? std::optional<std::vector<int>> {path}
                                     : std::nullopt;
    }
}


//std::optional<std::vector<int>> Graph::shortestPath(ResidualGraph &residual_graph) {
//    // Dijkstra with heap
//    std::vector<int> path (nVertices, 0), dist(nVertices, INF);
//    std::vector<bool> visited(nVertices, false); // if node is already optimal
//
//    dist[sourceNode] = 0;
//    // pair<int, int> => {distance between node u and source node , node u}
//    std::priority_queue<Edge, std::vector<Edge>, std::greater<>> q;
//    q.push(std::make_pair(0, sourceNode));
//
//    while (!q.empty()) {
//        auto [dist_u_to_src, u] = q.top(); q.pop();
//        if(visited[u]) continue; // it is already the optimal path
//        visited[u] = true;
//
//        for (int v = 0; v < nVertices; v++) {
//            if (v == u || v == sourceNode) continue;
//            int capacity = residual_graph[u][v];
//            int cost = adjacency_matrix[u][v].second;
//            if (capacity > 0 && dist[v] > dist_u_to_src + cost) {
//                dist[v] = dist[u] + cost;
//                q.push(std::make_pair(dist[v], v));
//                path[v] = u;
//            }
//        }
//    }
//
//    // check if the path exits
//    return dist[sinkNode] != INF ? std::optional<std::vector<int>> {path}
//                                 : std::nullopt;
//}
