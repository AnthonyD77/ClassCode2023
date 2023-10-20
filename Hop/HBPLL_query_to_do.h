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

    /**
     * TODO: Code that needs to be completed
     *
     *
     *
     */

    return distance;
}

vector<pair<int, int>> HB_extract_path_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst) {
    vector<pair<int, int>> paths;
    if (source == terminal) {
        return paths;
    }

    /**
     * TODO: Code that needs to be completed
     *
     *
     *
     */

    return paths;
}
