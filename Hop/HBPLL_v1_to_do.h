#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <Hop/HBPLL_two_hop_labels.h>

using namespace std;

struct HBPLL_v1_node {
public:
    int vertex, parent_vertex, hop;
    double priority_value;
};

bool operator<(HBPLL_v1_node const &x, HBPLL_v1_node const &y) {
    return x.priority_value > y.priority_value;
}
typedef typename boost::heap::fibonacci_heap<HBPLL_v1_node>::handle_type HBPLL_v1_node_handle;

void HB_thread_function_HBDIJ_Qhandle(int v_k, int N, int upper_k, bool use_rank_pruning) {
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
         * TODO: Code that needs to be completed
         *
         *
         *
         */
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

void HB_v1_sort_labels_thread(vector<vector<two_hop_label_v1>> *output_L, int v_k, double value_M) {
    sort(L_temp_599[v_k].begin(), L_temp_599[v_k].end(), compare_two_hop_label_small_to_large);
    if (value_M != 0) {
        int size_vk = L_temp_599[v_k].size();
        for (int i = 0; i < size_vk; i++) {
            L_temp_599[v_k][i].distance += L_temp_599[v_k][i].hop * value_M;
        }
    }
    (*output_L)[v_k] = L_temp_599[v_k];
    vector<two_hop_label_v1>().swap(L_temp_599[v_k]);  // clear new labels for RAM efficiency
}

vector<vector<two_hop_label_v1>> HB_v1_sort_labels(int N, int max_N_ID, int num_of_threads, double value_M = 0) {
    vector<vector<two_hop_label_v1>> output_L(max_N_ID);
    vector<vector<two_hop_label_v1>> *p = &output_L;

    ThreadPool pool(num_of_threads);
    std::vector<std::future<int>> results;
    for (int v_k = 0; v_k < N; v_k++) {
        results.emplace_back(pool.enqueue([p, v_k, value_M] {
            HB_v1_sort_labels_thread(p, v_k, value_M);
            return 1;
        }));
    }
    for (auto &&result : results)
        result.get();

    return output_L;
}

void HBPLL_v1(graph_v_of_v_idealID &input_graph, int num_of_threads, two_hop_case_info &case_info) {
    //----------------------------------- step 1: initialization -----------------------------------
    cout << "step 1: initialization" << endl;

    auto begin = std::chrono::high_resolution_clock::now();
    /* information prepare */
    begin_time_599 = std::chrono::high_resolution_clock::now();
    int N = input_graph.size();
    L_temp_599.resize(N);

    /* thread info */
    ThreadPool pool(num_of_threads);
    std::vector <std::future<int>> results;
    int num_of_threads_per_push = num_of_threads * 100;

    ideal_graph_599 = input_graph;
    auto end = std::chrono::high_resolution_clock::now();
    case_info.time_initialization = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;

    //----------------------------------------------- step 2: generate labels ---------------------------------------------------------------
    cout << "step 2: generate labels" << endl;
    begin = std::chrono::high_resolution_clock::now();

    /*searching shortest paths*/
    int upper_k = case_info.upper_k == 0 ? std::numeric_limits<int>::max() : case_info.upper_k;
    bool use_rank_pruning = case_info.use_rank_pruning;

    Temp_L_vk_599.resize(num_of_threads);
    dist_hop_599.resize(num_of_threads);
    for (int i = 0; i < num_of_threads; i++) {
        Temp_L_vk_599[i].resize(N);
        dist_hop_599[i].resize(N, {std::numeric_limits<double>::max(), 0});
        Qid_599.push(i);
    }

    int push_num = 0;
    for (int v_k = 0; v_k < N; v_k++) {
        if (ideal_graph_599[v_k].size() > 0) {
            results.emplace_back(
                    pool.enqueue([v_k, N, upper_k, use_rank_pruning] {
                        HB_thread_function_HBDIJ_Qhandle(v_k, N, upper_k, use_rank_pruning);
                        return 1;
                    }));
            push_num++;
        }
        if (push_num % num_of_threads_per_push == 0) {
            for (auto &&result: results)
                result.get();
            results.clear();
        }
    }

    for (auto &&result: results)
        result.get();

    end = std::chrono::high_resolution_clock::now();
    case_info.time_generate_labels = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;


    //----------------------------------------------- step 3: sort labels---------------------------------------------------------------
    cout << "step 3: sort labels" << endl;

    begin = std::chrono::high_resolution_clock::now();

    case_info.L = HB_v1_sort_labels(N, N, num_of_threads);

    end = std::chrono::high_resolution_clock::now();
    case_info.time_sort_labels = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;

    clear_global_values();
}