#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <Hop/HBPLL_two_hop_labels.h>

struct HBPLL_v1_node {
public:
    int vertex, parent_vertex, hop;
    double priority_value;
};

bool operator<(HBPLL_v1_node const &x, HBPLL_v1_node const &y) {
    return x.priority_value > y.priority_value;
}
typedef typename boost::heap::fibonacci_heap<HBPLL_v1_node>::handle_type HBPLL_v1_node_handle;

/**
 * Hop-Bounded Dijkstra, 在HBPLL生成索引过程中被并行调用
 * @param v_k                   遍历起点
 * @param N                     顶点总个数
 * @param upper_k               遍历边数上限, 即在一条路径上遍历超过了k条边则停止遍历, 一般情况下不会用到
 */
void HB_thread_function_HBDIJ_Qhandle(int v_k, int N, int upper_k) {
    /* get unique thread id */
    mtx_599[max_N_599 - 1].lock();
    int used_id = Qid_599.front();
    Qid_599.pop();
    mtx_599[max_N_599 - 1].unlock();

    /* Temp_L_vk_599 stores the label (dist and hop) of vertex v_k */
    queue<int> Temp_L_vk_changes;
    mtx_599[v_k].lock();
    int L_vk_size = L_temp_599[v_k].size();
    for (int i = 0; i < L_vk_size; i++) {
        int L_vk_vertex = L_temp_599[v_k][i].vertex;
        Temp_L_vk_599[used_id][L_vk_vertex].push_back({L_temp_599[v_k][i].distance, L_temp_599[v_k][i].hop});
        Temp_L_vk_changes.push(L_vk_vertex);
    }
    mtx_599[v_k].unlock();

    /* dist_hop_599 stores the shortest distance from vk to any other vertices with its hop_cst */
    queue<int> dist_hop_changes;
    dist_hop_599[used_id][v_k] = {0, 0};
    dist_hop_changes.push(v_k);
    /* {vertex, hop} -> ptr */
    map <pair<int, int>, pair<HBPLL_v1_node_handle, double>> Q_handle;
    boost::heap::fibonacci_heap <HBPLL_v1_node> Q;

    HBPLL_v1_node node;
    node.vertex = v_k;
    node.parent_vertex = v_k;
    node.hop = 0;
    node.priority_value = 0;
    Q_handle[{v_k, 0}] = {Q.push({node}), node.priority_value};

    two_hop_label_v1 xx;
    long long int new_label_num = 0;

    while (Q.size() > 0) {
        /**
         * TODO: Code needs to be completed
         *
         *
         *
         */
         cout << "Code needs to be completed" << endl;
    }


    while (Temp_L_vk_changes.size() > 0) {
        vector < pair < double, int >> ().swap(Temp_L_vk_599[used_id][Temp_L_vk_changes.front()]);
        Temp_L_vk_changes.pop();
    }

    while (dist_hop_changes.size() > 0) {
        dist_hop_599[used_id][dist_hop_changes.front()] = {std::numeric_limits<double>::max(), 0};
        dist_hop_changes.pop();
    }

    mtx_599[v_k].lock();
    vector<two_hop_label_v1>(L_temp_599[v_k]).swap(L_temp_599[v_k]);
    mtx_599[v_k].unlock();

    mtx_599[max_N_599 - 1].lock();
    Qid_599.push(used_id);
    mtx_599[max_N_599 - 1].unlock();
}


/**
 * 最短距离查询函数
 * @param L         索引集
 * @param source    查询起点
 * @param terminal  查询终点
 * @param hop_cst   边数边数(跳数)限制
 * @return          最短距离
 */
double HB_extract_distance_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst) {
    /*return std::numeric_limits<double>::max() is not connected*/
    if (hop_cst < 0) {
        return std::numeric_limits<double>::max();
    }
    if (source == terminal) {
        return 0;
    } else if (hop_cst == 0) {
        return std::numeric_limits<double>::max();
    }

    double distance = std::numeric_limits<double>::max();

    /**
     * TODO: Code needs to be completed
     *
     *
     *
     */

    return distance;
}


/**
 * 最短距离查询函数
 * @param L         索引集
 * @param source    查询起点
 * @param terminal  查询终点
 * @param hop_cst   查询边数(跳数)限制
 * @return          路径上边组成的集合, 如最短路径是 1->2->3 则返回 [{1,2},{2,3}]
 */
vector<pair<int, int>> HB_extract_path_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst) {
    vector<pair<int, int>> paths;
    if (source == terminal) {
        return paths;
    }

    /**
     * TODO: Code needs to be completed
     *
     *
     *
     */

    return paths;
}
