#pragma once

/*the following codes are for testing

---------------------------------------------------
a cpp file (try.cpp) for running the following test code:
----------------------------------------

#include <build_in_progress/HL/Hop/test_HBPLL.h>

int main() {
	test_HBPLL();
}

------------------------------------------------------------------------------------------
Commends for running the above cpp file on Linux:

g++ -std=c++17 -I/__PATH__/boost_1_75_0 -I/__PATH__/rucgraph run.cpp -lpthread -Ofast -o A
./A
rm A 

(optional to put the above commends in run.sh, and then use the comment: sh run.sh)

*/

#include <Hop/HBPLL.h>
#include <Hop/HBPLL_check_corectness.h>
#include <graph_v_of_v_idealID/random_graph/graph_v_of_v_idealID_generate_random_connected_graph.h>
#include <graph_v_of_v_idealID/read_save/graph_v_of_v_idealID_read.h>
#include <graph_v_of_v_idealID/read_save/graph_v_of_v_idealID_save.h>
#include <graph_v_of_v_idealID/graph_v_of_v_idaelID_sort.h>

void test_HBPLL() {
    /* Experimental parameters */
    int iteration_graph_times = 1e2;                        //总遍历次数, 即生成新的随机图测试的次数
    int thread_num = 5;                                     //HBPLL执行时的线程数
    int V = 5;                                             //生成新图的节点数
    int E = 7;                                             //生成新图的边数
    bool generate_new_graph = true;                         //是否生成新图,如果不生成则会读取之前生成的图,用于debug测试
    double ec_min = 0.1;                                    //生成新图的最小边权重
    double ec_max = 1;                                      //生成新图的最大边权重

    /* check parameters */
    bool check_correctness = true;                          //是否检查生成索引的正确性
    int iteration_source_times = 10;                        //检查正确性时起点随机生成的次数
    int iteration_terminal_times = 10;                      //检查正确性时终点随机生成的次数
    bool print_time_details = true;                         //是否打印HBPLL算法每个步骤的用时
    bool print_L = false;                                   //是否打印最终生成的索引

    /* hop bounded info */
    two_hop_case_info mm;

    /* result info */
    double avg_index_time = 0;
    double avg_index_size_per_v = 0;
    double avg_query_time = 0;
    double avg_canonical_repair_remove_label_ratio = 0;
    double total_time_initialization = 0;
    double total_time_generate_labels = 0;
    double total_time_canonical_repair = 0;

    /* iteration */
    for (int i = 0; i < iteration_graph_times; i++) {
        cout << ">>>iteration_graph_times: " << i << endl;

        graph_v_of_v_idealID instance_graph;
        if (generate_new_graph == 1) {
            instance_graph = graph_v_of_v_idealID_generate_random_connected_graph(V, E, ec_min, ec_max, 1, boost_random_time_seed);
            instance_graph = graph_v_of_v_idealID_sort(instance_graph);
            graph_v_of_v_idealID_save("simple_iterative_tests_HBPLL.txt", instance_graph);
        } else {
            graph_v_of_v_idealID_read("simple_iterative_tests_HBPLL.txt", instance_graph);
        }

        auto begin = std::chrono::high_resolution_clock::now();
        try {
            HBPLL(instance_graph, V, thread_num, mm);
            if (true) {
                total_time_initialization += mm.time_initialization;
                total_time_generate_labels += mm.time_generate_labels;
                total_time_canonical_repair += mm.time_canonical_repair;
            }
        } catch (string s) {
            cout << s << endl;
            clear_global_values();
            continue;
        }

        auto end = std::chrono::high_resolution_clock::now();

        double runningtime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
        avg_index_time = avg_index_time + runningtime / iteration_graph_times;

        if (print_L) {
            mm.print_L();
        }

        if (check_correctness) {
            HB_check_correctness(mm, instance_graph, iteration_source_times, iteration_terminal_times);
        }

        avg_query_time += mm.time_query;
        avg_index_size_per_v += (double)mm.compute_label_size() / V / iteration_graph_times;
        avg_canonical_repair_remove_label_ratio += (double) ((double)canonical_removed_labels / (double)mm.compute_label_size()) / iteration_graph_times;
        mm.clear_labels();
    }

    cout << "avg_index_time: " << avg_index_time << "s" << endl;
    cout << "avg_index_size_per_v: " << avg_index_size_per_v << endl;
    cout << "avg_canonical_repair_remove_label_ratio: " << avg_canonical_repair_remove_label_ratio << endl;
    if (check_correctness)
        cout << "avg_query_time: " << avg_query_time / (iteration_graph_times) << endl;
    if (print_time_details) {
        cout << "total_time_initialization: " << total_time_initialization << endl;
        cout << "total_time_generate_labels: " << total_time_generate_labels << endl;
        cout << "total_time_canonical_repair: " << total_time_canonical_repair << endl;
    }
}