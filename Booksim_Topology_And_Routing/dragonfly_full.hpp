#ifndef _DragonFly_Full_HPP_
#define _DragonFly_Full_HPP_

#include "network.hpp"
#include "routefunc.hpp"
#include "pair_hash.hpp"

#include <string>



class DragonFlyFull: public Network {
    int _a;
    int _g;
    int _h; 
    int _p;
    int _N; //no of routers 
    string _routing;    //this would be used to parameterise routing.
                        //so the same routing function can be used to behave differently
                        //based on the parameter passed from the config file.
    
                        //Accepted values for _routing: 
                                //min
                                //vlb 
                                //UGAL_L
                                //UGAL_L_two_hop
                                //UGAL_L_threshold

    string _ugal_multiply_mode;
                        //options: pathlen_based / one_vs_two
                        //During ugal path selection, 
                            //pathlen_based = > Q_min * len(min) <= Q_vlb * len(vlb)
                            //one_vs_two =>   Q_min * 1 <= Q_vlb * 2
    int _five_hop_percentage;

    
    int _radix; //router radix. = _a-1 + _h + _p
    
    int _threshold; //only needed for threshold based routing.
    
    int _local_latency;  //link latency. 
    int _global_latency;

    string _arrangement;
    
    void _setGlobals();
    void _setRoutingMode();
    void _AllocateArrays();
    void _ComputeSize(const Configuration &config);
    void _BuildNet(const Configuration &config);

    void _BuildGraphForLocal();
    void _BuildGraphForGlobal(string arrangement);
    void _CreatePortMap();
    
    void _discover_djkstra_paths();
    void _generate_two_hop_neighbors();
    void _generate_one_hop_neighbors();
    void _generate_common_neighbors_for_group_pair();


public:
    DragonFlyFull (const Configuration &config, const string & name);
    static void RegisterRoutingFunctions();
};

int select_shortest_path(int src_router, int dst_router, std::vector<int> & pathVector);
int select_shortest_path_djkstra(int src_router, int dst_router, std::vector<int> & pathVector);    
int find_port_to_node(int current_router, int next_router);

int allocate_vc(const Flit *f, int prev_router, int current_router, int next_router, int current_vc);

void min_djkstra_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject);

void min_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject);

/*
VLB routing
*/
void vlb_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject);

int select_vlb_path_old(int src_router, int dst_router, std::vector<int> & pathVector); //probably unnecessary

int select_vlb_path_regular(const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector);

int select_vlb_path(const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector);

int select_vlb_path_inside_group(int src_router, int dst_router, std::vector<int> & pathVector);


int vlb_intermediate_node_vanilla(const Flit *f, int src_router, int dst_router);


int vlb_imdt_node_for_four_hop_paths_old(const Flit *f, int src_router, int dst_router);
int vlb_imdt_node_for_four_hop_paths(const Flit *f, int src_router, int dst_router);
int vlb_imdt_node_for_three_hop_paths(const Flit *f, int src_router, int dst_router);


// These two both result in five_hop vlb paths, but work in slightly different way. 
// vlb_intermediate_node_two_hop_distance() only takes the neighbors that are
// two-hop away from the src, as i-nodes.

// vlb_imdt_node_for_five_hop_paths_src_and_dst() takes the union of neighbors that are
// two-hop away from the src or the dst, as i-nodes. So the number of paths
// will be increased in this case. 


int vlb_imdt_node_for_five_hop_paths_src_only(const Flit *f, int src_router, int dst_router);

int vlb_imdt_node_for_five_hop_paths_src_and_dst(const Flit *f, int src_router, int dst_router);

int vlb_imdt_node_for_four_hop_and_some_five_hop_paths(const Flit *f, int src_router, int dst_router, int five_hop_percentage);


int generate_vlb_path_from_given_imdt_node(int src_router, int dst_router, int imdt_router,  std::vector<int> & pathVector, string shortest_path_mode);

int generate_imdt_nodes(const Flit *f, int src_router, int dst_router, int no_of_nodes_to_generate, std::vector<int> & imdt_nodes, string routing_mode, int min_q_len);

/*
UGAL routing
*/
void UGAL_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject);

int select_UGAL_path(const Router *r, const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector, string routing_mode);

int make_UGAL_L_path_choice(const Router *r, const Flit *f, int no_of_min_paths_to_consider, int no_of_VLB_paths_to_consider, std::vector< std::vector<int> > paths, int chosen_tier);

int make_UGAL_G_path_choice(const Router *r, const Flit *f, int no_of_min_paths_to_consider, int no_of_VLB_paths_to_consider, std::vector< std::vector<int> > paths, int chosen_tier);

int find_port_queue_len_to_node(const Router *r, int current_router, int next_router);

int generate_in_group_vlb_paths(const Flit *f, int src_router, int dst_router, int no_of_nodes_to_generate, std::vector< std::vector<int> >::iterator start, std::vector< std::vector<int> >::iterator finish);



/*
PAR routing
*/
void PAR_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject);

//just for testing
void print_2d_vector(std::vector < std::vector <int> > &vect);

template <typename T>
void print_container(T &container);

template <typename T>
void fisher_yates_shuffle(T &container, int size);

//for ugal path comparison stat collection
void log_ugal_stats(int min_or_vlb_choice, int min_multiplier, int vlb_multiplier);

#endif

