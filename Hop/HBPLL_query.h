#pragma once

#include <Hop/HBPLL_two_hop_labels.h>

double HB_extract_distance(vector <vector<two_hop_label_v2>> &L2, int source, int terminal, int hop_cst) {
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
    auto vector1_check_pointer = L2[source].begin();
    auto vector2_check_pointer = L2[terminal].begin();
    auto pointer_L_s_end = L2[source].end(), pointer_L_t_end = L2[terminal].end();

    while (vector1_check_pointer != pointer_L_s_end && vector2_check_pointer != pointer_L_t_end) {
        if (vector1_check_pointer->vertex == vector2_check_pointer->vertex) {
            // in the common vertex
            auto ptr1 = vector1_check_pointer->dist_info.end() - 1;
            auto ptr2 = vector2_check_pointer->dist_info.end() - 1;
            auto begin1 = vector1_check_pointer->dist_info.begin();
            auto begin2 = vector2_check_pointer->dist_info.begin();
            while (1) {
                if (get<0>(*ptr1) != -1) {
                    if (get<2>(*ptr1) + get<2>(*ptr2) <= hop_cst && get<0>(*ptr2) != -1) {
                        double dis = get<0>(*ptr1) + get<0>(*ptr2);
                        if (distance > dis) {
                            distance = dis;
                        }
                        break;
                    }
                    if (ptr2 != begin2) {
                        ptr2--;
                        while (1) {
                            if (get<0>(*ptr2) != -1) {
                                if (get<2>(*ptr1) + get<2>(*ptr2) <= hop_cst) {
                                    double dis = get<0>(*ptr1) + get<0>(*ptr2);
                                    if (distance > dis) {
                                        distance = dis;
                                    }
                                    break;
                                }
                            }
                            if (ptr2 == begin2) {
                                break;
                            }
                            ptr2--;
                        }
                    }
                }
                if (ptr1 == begin1) {
                    break;
                }
                ptr1--;
                ptr2 = vector2_check_pointer->dist_info.end() - 1;
            }
            vector1_check_pointer++;
        } else if (vector1_check_pointer->vertex > vector2_check_pointer->vertex) {
            vector2_check_pointer++;
        } else {
            vector1_check_pointer++;
        }
    }

    return distance;
}

vector <pair<int, int>> HB_extract_path(vector <vector<two_hop_label_v2>> &L2, int source, int terminal, int hop_cst) {
    vector <pair<int, int>> paths;
    if (source == terminal) {
        return paths;
    }

    double min_dis = std::numeric_limits<double>::max();
    vector <pair<int, int>> partial_edges(2);

    int vector1_capped_v_parent = 0, vector2_capped_v_parent = 0;
    double distance = std::numeric_limits<double>::max();
    bool connected = false;
    auto vector1_check_pointer = L2[source].begin();
    auto vector2_check_pointer = L2[terminal].begin();
    auto pointer_L_s_end = L2[source].end(), pointer_L_t_end = L2[terminal].end();

    while (vector1_check_pointer != pointer_L_s_end && vector2_check_pointer != pointer_L_t_end) {
        if (vector1_check_pointer->vertex == vector2_check_pointer->vertex) {
            auto ptr1 = vector1_check_pointer->dist_info.end() - 1;
            auto ptr2 = vector2_check_pointer->dist_info.end() - 1;
            auto begin1 = vector1_check_pointer->dist_info.begin();
            auto begin2 = vector2_check_pointer->dist_info.begin();
            while (1) {
                if (get<0>(*ptr1) != -1) {
                    if (get<2>(*ptr1) + get<2>(*ptr2) <= hop_cst && get<0>(*ptr2) != -1) {
                        connected = true;
                        double dis = get<0>(*ptr1) + get<0>(*ptr2);
                        if (distance > dis) {
                            distance = dis;
                            vector1_capped_v_parent = get<1>(*ptr1);
                            vector2_capped_v_parent = get<1>(*ptr2);
                        }
                        break;
                    }
                    if (ptr2 != begin2) {
                        ptr2--;
                        while (1) {
                            if (get<0>(*ptr2) != -1) {
                                if (get<2>(*ptr1) + get<2>(*ptr2) <= hop_cst) {
                                    connected = true;
                                    double dis = get<0>(*ptr1) + get<0>(*ptr2);
                                    if (distance > dis) {
                                        distance = dis;
                                        vector1_capped_v_parent = get<1>(*ptr1);
                                        vector2_capped_v_parent = get<1>(*ptr2);
                                    }
                                    break;
                                }
                            }
                            if (ptr2 == begin2) {
                                break;
                            }
                            ptr2--;
                        }
                    }
                }
                if (ptr1 == begin1) {
                    break;
                }
                ptr1--;
                ptr2 = vector2_check_pointer->dist_info.end() - 1;
            }
            vector1_check_pointer++;
        } else if (vector1_check_pointer->vertex > vector2_check_pointer->vertex) {
            vector2_check_pointer++;
        } else {
            vector1_check_pointer++;
        }
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

    vector <pair<int, int>> new_edges;
    new_edges = HB_extract_path(L2, source, terminal, hop_cst);
    if (new_edges.size() > 0) {
        for (int i = new_edges.size() - 1; i >= 0; i--) {
            paths.push_back(new_edges[i]);
        }
    }


    return paths;
}
