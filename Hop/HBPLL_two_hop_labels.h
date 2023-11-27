#pragma once
#include <map>
#include <shared_mutex>
#include <tool_functions/ThreadPool.h>
#include <graph_v_of_v_idealID/graph_v_of_v_idealID.h>

using namespace std;

/* label format */
class two_hop_label_v1 {
public:
    int vertex, parent_vertex;
    int hop;
    double distance;
};

/*
    global values
    unique code for this file: 599
*/
long long int max_N_599 = 1e7;
vector<std::shared_timed_mutex> mtx_599(max_N_599);
graph_v_of_v_idealID ideal_graph_599;
vector<vector<two_hop_label_v1>> L_temp_599;
vector<vector<vector<pair<double, int>>>> Temp_L_vk_599;
vector<vector<pair<double, int>>> dist_hop_599;
queue<int> Qid_599;

void clear_global_values() {
    vector<vector<two_hop_label_v1>>().swap(L_temp_599);
    vector<vector<vector<pair<double, int>>>>().swap(Temp_L_vk_599);
    vector<vector<pair<double, int>>>().swap(dist_hop_599);
    ideal_graph_599.clear();
    queue<int>().swap(Qid_599);
}

class two_hop_case_info {
public:
    /*hop bounded*/
    int upper_k = 10;

    /*running time records*/
    double time_initialization = 0;
    double time_generate_labels = 0;
    double time_sort_labels = 0;
    double time_query = 0;

    /*labels*/
    vector<vector<two_hop_label_v1>> L;

    long long int compute_label_size() {
        long long int size = 0;
        for (auto it = L.begin(); it != L.end(); it++) {
            for(auto it2 = it->begin(); it2 != it->end(); it2++) {
                size++;
            }
        }
        return size;
    }

    /*clear labels*/
    void clear_labels() {
        vector<vector<two_hop_label_v1>>().swap(L);
        time_initialization = 0;
        time_generate_labels = 0;
        time_sort_labels = 0;
        time_query = 0;
    }

    /*printing*/
    void print_L() {
        cout << "print_L:" << endl;
        for (int i = 0; i < L.size(); i++) {
            cout << "L[" << i << "]=";
            for (int j = 0; j < L[i].size(); j++) {
                cout << "{" << L[i][j].vertex << "," << L[i][j].distance << "," << L[i][j].parent_vertex << ","
                     << L[i][j].hop << "}";
            }
            cout << endl;
        }
    }

    void print_L_vk(int v_k) {
        for (auto it = L[v_k].begin(); it != L[v_k].end(); it++) {
            cout << "<" << it->vertex << "," << it->distance << "," << it->parent_vertex << "," << it->hop << ">";
        }
        cout << endl;
    }
};

bool compare_two_hop_label_small_to_large(two_hop_label_v1 &i, two_hop_label_v1 &j) {
    if (i.vertex != j.vertex) {
        return i.vertex < j.vertex;
    } else if (i.hop != j.hop) {
        return i.hop < j.hop;
    } else {
        return i.distance > j.distance;
    }
}
