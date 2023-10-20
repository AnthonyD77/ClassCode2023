#pragma once

/*the following codes are for testing

---------------------------------------------------
a cpp file (try.cpp) for running the following test code:
----------------------------------------

#include <build_in_progress/HL/Hop/test_HBPLL.h>

int main()
{
	test_HBPLL();
}

------------------------------------------------------------------------------------------
Commends for running the above cpp file on Linux:

g++ -std=c++17 -I/__PATH__/boost_1_75_0 -I/__PATH__/rucgraph run.cpp -lpthread -Ofast -o A
./A
rm A 

(optional to put the above commends in run.sh, and then use the comment: sh run.sh)

*/

#include <Hop/graph_v_of_v_idealID_HB_v1.h>
#include <Hop/graph_v_of_v_idealID_HB_check_corectness.h>
#include <graph_v_of_v_idealID/random_graph/graph_v_of_v_idealID_generate_random_connected_graph.h>
#include <graph_v_of_v_idealID/read_save/graph_v_of_v_idealID_read.h>
#include <graph_v_of_v_idealID/read_save/graph_v_of_v_idealID_save.h>
#include <graph_v_of_v_idealID/graph_v_of_v_idaelID_sort.h>

void test_HBPLL() {
    /* graph parameters */
    int iteration_graph_times = 1e2, iteration_source_times = 1000, iteration_terminal_times = 1000;
    int V = 20, E = 50, precision = 1, thread_num = 1;
    double ec_min = 0.1, ec_max = 1;

    /* test parameters */
    bool generate_new_graph = true;
    bool use_2019R2 = false, use_enhanced2019R2 = false, use_non_adj_reduc_degree = false;
    bool use_rank_pruning = true;
    bool use_canonical_repair = true;
    bool use_M = false;
    int upper_k = 10;

    bool print_time_details = true;
    bool print_label_before_canonical_fix = false;
    bool print_L = false;
    bool check_correctness = true;
    check_path = true;

    debug = 0;
    if (debug) {
        source_debug = 0;
        terminal_debug = 5;
        hop_cst_debug = 2;
        generate_new_graph = 0;
        iteration_graph_times = 1;
        iteration_source_times = 1;
        iteration_terminal_times = 1;
        print_L = 1;
    }

    /* hop bounded info */
    graph_v_of_v_idealID_two_hop_case_info_v1 mm;
    int query_hop_cst = 5;
    mm.use_rank_pruning = use_rank_pruning;
    mm.value_M = use_M ? ec_max * E : 0;
    mm.upper_k = upper_k;
    mm.use_2019R2 = use_2019R2;
    mm.use_enhanced2019R2 = use_enhanced2019R2;
    mm.use_non_adj_reduc_degree = use_non_adj_reduc_degree;
    mm.print_label_before_canonical_fix = print_label_before_canonical_fix;
    mm.use_canonical_repair = use_canonical_repair;

    /* result info */
    double avg_index_time = 0, avg_index_size_per_v = 0, avg_MG_num = 0;
    double avg_query_time = 0, avg_canonical_repair_remove_label_ratio = 0;
    double total_time_initialization = 0, total_time_reduction = 0, total_time_generate_labels = 0;
    double total_time_update_predecessors = 0, total_time_canonical_repair = 0;

    /* iteration */
    for (int i = 0; i < iteration_graph_times; i++) {
        cout << ">>>iteration_graph_times: " << i << endl;

        graph_v_of_v_idealID instance_graph;
        if (generate_new_graph == 1) {
            instance_graph = graph_v_of_v_idealID_generate_random_connected_graph(V, E, ec_min, ec_max, precision, boost_random_time_seed);
            instance_graph = graph_v_of_v_idealID_sort(instance_graph);
            graph_v_of_v_idealID_save("simple_iterative_tests_HBPLL.txt", instance_graph);
        } else {
            graph_v_of_v_idealID_read("simple_iterative_tests_HBPLL.txt", instance_graph);
        }

        auto begin = std::chrono::high_resolution_clock::now();
        try {
            graph_v_of_v_idealID_HB_v2(instance_graph, V, thread_num, mm);
            if (print_time_details) {
                total_time_initialization += mm.time_initialization;
                total_time_reduction += mm.time_reduction;
                total_time_generate_labels += mm.time_generate_labels;
                total_time_update_predecessors += mm.time_update_predecessors;
                total_time_canonical_repair += mm.time_canonical_repair;
            }
        } catch (string s) {
            cout << s << endl;
            graph_v_of_v_idealID_two_hop_clear_global_values();
            continue;
        }

        cout << "finish generating labels" << endl;

        auto end = std::chrono::high_resolution_clock::now();

        double runningtime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;  // s
        avg_index_time = avg_index_time + runningtime / iteration_graph_times;
        avg_MG_num = avg_MG_num + (double) mm.MG_num / iteration_graph_times;

        if (print_L)
            mm.print_L();

        if (check_correctness) {
            graph_v_of_v_idealID_HB_v2_check_correctness(mm, instance_graph, iteration_source_times, iteration_terminal_times, query_hop_cst);
        }

        avg_query_time += mm.time_query;
        avg_index_size_per_v += (double)mm.compute_label_size() / V / iteration_graph_times;
        avg_canonical_repair_remove_label_ratio += (double) ((double)canonical_removed_labels / (double)mm.compute_label_size()) / iteration_graph_times;

        graph_v_of_v_idealID_two_hop_clear_global_values2();
        mm.clear_labels();
    }

    cout << "avg_index_time: " << avg_index_time << "s" << endl;
    cout << "avg_index_size_per_v: " << avg_index_size_per_v << endl;
    if (mm.use_2019R2 || mm.use_enhanced2019R2 || mm.use_non_adj_reduc_degree)
        cout << "avg_MG_num: " << avg_MG_num << endl;
    if (mm.use_canonical_repair)
        cout << "avg_canonical_repair_remove_label_ratio: " << avg_canonical_repair_remove_label_ratio << endl;
    if (check_correctness)
        cout << "avg_query_time: " << avg_query_time / (iteration_graph_times) << endl;
    if (print_time_details) {
        cout << "\t total_time_initialization: " << total_time_initialization << endl;
        cout << "\t total_time_reduction: " << total_time_reduction << endl;
        cout << "\t total_time_generate_labels: " << total_time_generate_labels << endl;
        cout << "\t total_time_update_predecessors: " << total_time_update_predecessors << endl;
        cout << "\t total_time_canonical_repair: " << total_time_canonical_repair << endl;
    }
}
