#pragma once

#include <Hop/HBPLL_two_hop_labels.h>

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
    auto vector1_check_pointer = L[source].begin();
    auto vector2_check_pointer = L[terminal].begin();
    auto pointer_L_s_end = L[source].end(), pointer_L_t_end = L[terminal].end();
    while (vector1_check_pointer != pointer_L_s_end) {
        vector2_check_pointer = L[terminal].begin();
        while (vector2_check_pointer != pointer_L_t_end) {
            if (vector2_check_pointer->vertex > vector1_check_pointer->vertex)
                break;
            if (vector1_check_pointer->vertex == vector2_check_pointer->vertex) {
                if (vector1_check_pointer->hop + vector2_check_pointer->hop <= hop_cst) {
                    double dis = vector1_check_pointer->distance + vector2_check_pointer->distance;
                    if (distance > dis) {
                        distance = dis;
                    }
                }
            }
            vector2_check_pointer++;
        }
        vector1_check_pointer++;
    }

    return distance;
}

vector<pair<int, int>> HB_extract_path_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst) {
    vector<pair<int, int>> paths;
    if (source == terminal) {
        return paths;
    }

    vector<pair<int, int>> partial_edges(2);
    
    int vector1_capped_v_parent = 0, vector2_capped_v_parent = 0;
    double distance = std::numeric_limits<double>::max();
    bool connected = false;
    auto vector1_check_pointer = L[source].begin();
    auto vector2_check_pointer = L[terminal].begin();
    auto pointer_L_s_end = L[source].end(), pointer_L_t_end = L[terminal].end();
    while (vector1_check_pointer != pointer_L_s_end) {
        vector2_check_pointer = L[terminal].begin();
        while (vector2_check_pointer != pointer_L_t_end) {
            if (vector2_check_pointer->vertex > vector1_check_pointer->vertex)
                break;
            if (vector1_check_pointer->vertex == vector2_check_pointer->vertex) {
                if (vector1_check_pointer->hop + vector2_check_pointer->hop <= hop_cst) {
                    connected = true;
                    double dis = vector1_check_pointer->distance + vector2_check_pointer->distance;
                    if (distance > dis) {
                        distance = dis;
                        vector1_capped_v_parent = vector1_check_pointer->parent_vertex;
                        vector2_capped_v_parent = vector2_check_pointer->parent_vertex;
                    }
                }
            }
            vector2_check_pointer++;
        }
        vector1_check_pointer++;
    }

    if (connected) {
        if (source != vector1_capped_v_parent) {
            paths.push_back({source, vector1_capped_v_parent});
            source = vector1_capped_v_parent;
            hop_cst--;
        }
        if (terminal != vector2_capped_v_parent) {
            paths.push_back({terminal, vector2_capped_v_parent});
            terminal = vector2_capped_v_parent;
            hop_cst--;
        }
    } else {
        return paths;
    }

    /* find new */
    vector<pair<int, int>> new_edges;
    new_edges = HB_extract_path_v1(L, source, terminal, hop_cst);

    if (new_edges.size() > 0) {
        for (int i = new_edges.size() - 1; i >= 0; i--) {
            paths.push_back(new_edges[i]);
        }
    }
    return paths;
}
