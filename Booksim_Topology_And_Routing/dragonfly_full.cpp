#include "booksim.hpp"

#include <vector>
#include <unordered_map>
#include <sstream>
#include <unordered_set>
#include <algorithm>

#include <fstream>

#include "random_utils.hpp"
#include "combinations.hpp"

#include "dragonfly_full.hpp"

#include "djkstra.hpp"
#define INF 9999    
    //this is critical for djkstra to work. Don't change it.

#include "global_stats.hpp"

//#define DRAGON_LATENCY


using namespace std;


#define FLIT_TO_TRACK -1

/*
Data types:

We need:
- a graph to hold the topology and wieght of each link. 
- a map for each node to quickly find the port map to its neighbor nodes.

So here are the data types we are going to used:

g_graph: a 2D vector of type int, of dimension N * node_degree. 
        Contains the neighbor of each node
    
g_link_weights: a 2D vector of type int, of dimension N * node_degree
        Contains the weight of the link.
        Weight is basically the type of links, used in routing.
        By default, weight is 1 for local links, and 3 for global links. 

g_link_widths: a 2D vector of type int, of dimension N * node_degree
        Contains the width of the link.
        Width means how many links go between that SD pair.
        Usually its 1, but depedning on link arrangement policy, it can be mroe than 1.

g_global_link_frequency: an unordered_map that uses the (src,dest) pair as key 
                and its width as value.
                This kind of makes g_link_widths redundant, but lets have both. Juse because.        

So, 
g_graph[0][5] = 10 means node 0 is connected to node 10 as its sixth neighbor.
g_link_weights[0][5] = 3 means the weight of the link between node 0 and its 
                        sixth neighbor (node 10) is 3.
g_link_widths[0][5] = 2 means the width of the link between node 0 and its 
                        sixth neighbor (node 10) is 2.
                        
These two arrays could've been glued together, but I think this is cleaner.


g_port_map: a unordered_map that uses the (src,dest) pair as key and 
            [start_port, end_port] pair as value.
            
so g_port_map[(0,5)] = (5,6) means node 0 is connected to node 5, and there are
            two output ports, ports 5 and 6, from node 0 to 5.



All these structures are kept as global so that the routing functions can use them.

*/


//dragonfly parameters. we need to access some of them from the routing function.
//so making them global.
int g_a;
int g_g;
int g_h;
int g_p;
int g_N;
int g_threshold;
int g_log_Qlen_data;

#define LOCAL_LINK_WEIGHT 1
#define GLOBAL_LINK_WEIGHT 3

string g_routing_function;

string g_routing_mode; 
string g_ugal_multiply_mode;
string g_ugal_local_vs_global_switch;
int g_five_hop_percentage;

string g_vc_allocation_mode;
    
std::vector < std::vector <int> > g_graph;
std::vector < std::vector <int> > g_link_weights;
std::vector < std::vector <int> > g_link_widths;

//std::vector < std::unordered_map< int, std::pair<int,int> > > g_port_map; 

std::unordered_map < std::pair<int, int>, int, pair_hash > g_global_link_frequency;

std::unordered_map< std::pair<int, int>, std::pair<int,int>, pair_hash >  g_port_map; 

//g_distance and g_parents are populated by djkstra; will be used in djkstra routing
std::vector< std::vector< int >> g_distance;
std::vector< std::vector< std::vector<int> > > g_parents;

//We need all the channels between groups for routing
std::vector< std::vector < std::vector <std::pair<int,int> > > > g_inter_group_links;

//For 4-hop-vlb-path generation, we need a list of i-nodes connected to each
//pair of groups.

//std::vector< std::vector <std::vector <int> >> g_group_pair_vs_common_nodes;

std::unordered_map < std::pair<int, int>, std::vector<int>, pair_hash > g_group_pair_vs_common_nodes;


//data structres needed to 2-hop neighbor cache.
std::vector < std::unordered_set <int> > two_hop_neighbors_set;
std::vector < std::vector <int> > two_hop_neighbors_vector;

//data structure needed for 1-hop neighbor cache
std::vector < std::unordered_set <int> > one_hop_neighbors_set;
std::vector < std::vector <int> > one_hop_neighbors_vector;



//flie handle to write Q_len data
ofstream g_q_len_file;


// For Ugal_g, we need to keep a global array of all the router objects
std::vector <Router *> all_routers;

// These are to keep stats of multi-tiered routing.

DragonFlyFull::DragonFlyFull(const Configuration &config, const string &name) : Network(config, name){
    
    cout << "inside DragonFlyFull() constructor ..." << endl;

    //trafficmanager initializes the seed. 
    //This block is only needed if we want to play with the constructor and test/debug something.
    /*int seed = config.GetInt("seed");
    RandomSeed(seed);*/

    //these three needs to be parameterized later
    //_a = 4;
    //_g = 4;
    //_arrangement = "absolute_improved";
    _a = config.GetInt("df_a");
    _g = config.GetInt("df_g");
    _arrangement = config.GetStr("df_arrangement");
    
    _routing = config.GetStr("routing_function");


    _threshold = config.GetInt("routing_threshold");

    _local_latency = config.GetInt("local_latency");
    _global_latency = config.GetInt("global_latency");

    _ugal_multiply_mode = config.GetStr("ugal_multiply_mode");

    _five_hop_percentage = config.GetInt("five_hop_percentage");

 
    g_log_Qlen_data = config.GetInt("log_Qlen_data");
    
    g_vc_allocation_mode = config.GetStr("vc_allocation_mode");
    
    if (g_log_Qlen_data == 1){

        // current date/time based on current system
        time_t now = time(0);
        tm *ltm = localtime(&now);

        int year, month, day, hour, min, sec;

        year  = 1900 + ltm->tm_year;
        month = 1 + ltm->tm_mon;
        day = ltm->tm_mday;
        hour = ltm->tm_hour;
        min = ltm->tm_min;
        sec = ltm->tm_sec;
       
        int i_rate = int(config.GetFloat("injection_rate")*100); 
        string q_len_file_name = "QLenData/Qdata_" + std::to_string(_a) + "_" +  std::to_string(_g) + "_" + _routing + "_" + config.GetStr("traffic") + "_" + std::to_string(i_rate) +  "_" + std::to_string(year) + "_" + std::to_string(month) +"_" + std::to_string(day) + "_" + std::to_string(hour)  + "_" + std::to_string(min) + "_" + std::to_string(sec) +".qdata";
        cout << "q_len_file_name: " << q_len_file_name << endl;   
        ed_q_len_file.open(q_len_file_name); 

        if (ed_q_len_file.is_open() == false){
            cout << "Error opening Qlen_data file!" << endl;
        }
    }

    //just going with the default settings. No immediate plan to change.
    _h = _a/2;
    _p = _a/2;
    
    _N = _a * _g;
    _radix = _a - 1 + _h + _p; //channels to intra-group routers + inter-group routers + to PEs
    
    cout << "_a: " << _a << "  _g: " << _g << "  _h: " << _h << "  _p: " << _p << "  _arrangement: "  << _arrangement << endl; 
    
    cout << "local link latency: " << _local_latency << endl;
    cout << "global link latency: " << _global_latency << endl;
    cout << "latency threshold: " << config.GetFloat("latency_thres") << endl;

    cout << "Routing function: " << _routing << endl;
    cout << "ugal multiply mode: " << _ugal_multiply_mode << endl;

    cout << "vc_allocation_mode: " << g_vc_allocation_mode << endl;
    
    // cout << "Routing threshold: " << _threshold << endl;




    _setGlobals();
    _setRoutingMode();
    _AllocateArrays();
    _BuildGraphForLocal();
    _BuildGraphForGlobal(_arrangement);
    _CreatePortMap();
    
    _ComputeSize( config );
    _Alloc( );
    _BuildNet( config );
    
    _discover_djkstra_paths();
    _generate_one_hop_neighbors();
    _generate_two_hop_neighbors();

    //this is only required for restricted routing where 4_hop paths are used. 
    //set a conditional accordingly.
    _generate_common_neighbors_for_group_pair();

    cout << "Done with DragonFlyFull() constructor ..." << endl;
    
    //exit(-1);
    //for test
    /*int src_router = 0;
    int dst_router = 10;
    int min_q_len = 55; 
    Flit * f  = Flit::New();

 
    exit(-1);*/
}

void print_2d_vector(std::vector < std::vector <int> > &vect){
    std::size_t ii, jj; 
    
    //cout << "vector contents:" << endl;
    for ( ii = 0; ii < vect.size(); ii++){
        cout << ii << " : ";
        for (jj=0; jj < vect[ii].size(); jj++){
            cout << vect[ii][jj] << " ";
        }
        cout << endl;
    }
    //cout << "printing done." << endl;
    
}

//mostly to print vectors, and may be sets as well.
template <typename T>
void print_container(T &container){
    for (auto it = container.begin(); it != container.end(); it++){
        cout << *it << " ";
    }
    cout << endl;
}

    
void test_vector_of_maps(std::vector < std::unordered_map< int, std::pair<int,int> > > &vect_map){
    //randomly add some elements in the vector map
    vect_map[10][5] = std::pair<int, int>(50,55);
    vect_map[10][7] = std::pair<int, int>(70,77);
    vect_map[2][3] = std::pair<int, int>(30,33);
    vect_map[6][1] = std::make_pair(10,11);
    
    std::size_t ii;
    cout << "vector contents:" << endl;
    for(ii=0; ii < vect_map.size(); ii++){
        cout << ii << " : ";
        for(auto it = vect_map[ii].begin(); it != vect_map[ii].end(); it++){
            cout << "<" << it->first << " , " << it->second.first << "," << it->second.second << ">  ";
        }
        cout << endl;
    }
    cout << "printing done." << endl;
    
}

void DragonFlyFull::_setGlobals(){
    /*
    dragonfly parameters. we need to access some of them from the routing function.
    so made them global.
    */
    g_a = _a;
    g_g = _g;
    g_h = _h;
    g_p = _p;
    g_N = _N;
    
    g_routing_function = _routing;
    g_five_hop_percentage = _five_hop_percentage;
    
    g_threshold = _threshold;

    //stat collection variables
    g_total_flit = 0;
    g_total_min_flit = 0;
    g_total_non_min_flit = 0;

    for(int ii = 0; ii < MAX_PATH_LEN; ii++){
        g_min_ingroup_path_dist[ii] = 0;
        g_min_outgroup_path_dist[ii] = 0;
    }
    for(int ii = 0; ii < MAX_PATH_LEN*2; ii++){
        g_vlb_ingroup_path_dist[ii] = 0;
        g_vlb_outgroup_path_dist[ii] = 0;
    }

    for(int ii = 0; ii < MAX_CASE; ii++){
        g_cases_when_not_taken[ii] = 0;
        g_cases_when_taken[ii] = 0;
        g_lens_when_not_taken[ii] = 0;
        g_lens_when_taken[ii] = 0;
    }   

}

void DragonFlyFull::_setRoutingMode(){
    
    //possible supported routings: 
        //min/ vlb/ UGAL_L/ UGAL_L_two_hop / UGAL_L_threshold/
        // PAR
    if ((_routing == "UGAL_G") || (_routing == "UGAL_G_restricted_src_only") || (_routing == "UGAL_G_restricted_src_and_dst") || (_routing == "UGAL_G_four_hop_restricted") || (_routing == "UGAL_G_four_hop_some_five_hop_restricted") || (_routing == "UGAL_G_three_hop_restricted")){
        g_ugal_local_vs_global_switch = "global";

    } else if ( (_routing == "UGAL_L")  || (_routing == "UGAL_L_two_hop") || (_routing == "UGAL_L_restricted_src_only") || (_routing == "UGAL_L_restricted_src_and_dst") || (_routing == "UGAL_L_threshold")   || (_routing == "UGAL_L_four_hop_restricted") ||  (_routing == "UGAL_L_four_hop_some_five_hop_restricted") || (_routing == "UGAL_L_three_hop_restricted") || (_routing == "PAR") || (_routing == "PAR_restricted_src_only") || (_routing == "PAR_restricted_src_and_dst") || (_routing == "PAR_four_hop_restricted")|| (_routing == "PAR_four_hop_some_five_hop_restricted") || (_routing == "PAR_three_hop_restricted") ) {
        g_ugal_local_vs_global_switch = "local";

    } else{
        g_ugal_local_vs_global_switch = "not_applicable";
    }

    //possible modes: vanilla, two_hop, threshold,  not_applicable

    if ((_routing == "vlb") || (_routing == "UGAL_L") || (_routing == "UGAL_G") || (_routing == "PAR")) {
        g_routing_mode = "vanilla";
    } 
    else if (_routing == "UGAL_L_two_hop"){
        g_routing_mode = "two_hop";    
    }
    else if ( (_routing == "vlb_restricted_src_only") || (_routing == "UGAL_L_restricted_src_only") || (_routing == "UGAL_G_restricted_src_only") || (_routing == "PAR_restricted_src_only") ){
        g_routing_mode = "restricted_src_only";    
    }
    else if ( (_routing == "vlb_restricted_src_and_dst") || (_routing == "UGAL_L_restricted_src_and_dst") || (_routing == "UGAL_G_restricted_src_and_dst") || (_routing == "PAR_restricted_src_and_dst") ){
        g_routing_mode = "restricted_src_and_dst";    
    }
    else if ( (_routing == "vlb_four_hop_restricted") || (_routing == "UGAL_L_four_hop_restricted") || (_routing == "UGAL_G_four_hop_restricted") || (_routing == "PAR_four_hop_restricted")  ){
        g_routing_mode = "four_hop_restricted";    
    }
    else if (  (_routing == "UGAL_L_three_hop_restricted") || (_routing == "UGAL_G_three_hop_restricted") || (_routing == "PAR_three_hop_restricted")  ){
        g_routing_mode = "three_hop_restricted";   
    } 
    else if ( (_routing == "vlb_four_hop_some_five_hop_restricted") || (_routing == "UGAL_L_four_hop_some_five_hop_restricted") || (_routing == "UGAL_G_four_hop_some_five_hop_restricted") || (_routing == "PAR_four_hop_some_five_hop_restricted")  ){
        g_routing_mode = "four_hop_some_five_hop_restricted";    
    }
    else if (_routing == "UGAL_L_threshold"){
        g_routing_mode = "threshold";    
    }
  
    else{
        g_routing_mode = "not_applicable";    
    }


    //set ugal_multiply_mode
    g_ugal_multiply_mode = _ugal_multiply_mode;

}


void DragonFlyFull::_AllocateArrays(){
    /*
    allocate the vectors:
        std::vector < std::vector <int> > g_graph;
        std::vector < std::vector <int> > g_link_weights;
        //std::vector < std::unordered_map< int, std::pair<int,int> > > g_port_map; 

    Each has a length of _N => number of routers 
    g_graph and g_link_weights each have a row leght of router degree = _(_h + _a - 1)
    
    */
    
    cout << "inside _AllocateArrays() ... " << endl;
    
    g_graph.resize(_N);
    for (std::size_t ii=0; ii < g_graph.size(); ii++){
        g_graph[ii] = std::vector<int> (_h + _a - 1, -1);  //-1 default value
    }
    
    g_link_weights.resize(_N);
    for (std::size_t ii=0; ii < g_link_weights.size(); ii++){
        g_link_weights[ii] = std::vector<int> (_h + _a - 1, -1);  //-1 default value
    }
    
    g_link_widths.resize(_N);
    for (std::size_t ii=0; ii < g_link_widths.size(); ii++){
        g_link_widths[ii] = std::vector<int> (_h + _a - 1, -1);  //-1 default value
    }
    
    g_inter_group_links.resize(_g, std::vector< std::vector <std::pair<int,int>> >  (_g) );

    //g_group_pair_vs_common_nodes.resize(_g, std::vector < std::vector <int>> (_g));
    
    /*g_port_map.resize(_N);
    for( int ii = 0; ii < g_port_map.size(); ii++ ){
        g_port_map[ii] = std::unordered_map< int, std::pair<int,int> > ();
    }*/
        
    
    //print_2d_vector(g_link_weights);
    //test_vector_of_maps(g_port_map);
    
    cout << "done with _AllocateArrays() ... " << endl;
    
}
    
void DragonFlyFull::_ComputeSize(const Configuration &config){
    cout << "_Computesize starts ..." << endl;
    
    //values needs to be provided
    
    //_nodes -> no of PEs
    _nodes = _a * _g * _p;
    cout << "_nodes: " << _nodes << endl;
    
    //_size -> no of routers
    _size = _a * _g;
    cout << "_size: " << _size << endl;
    
    //_channels -> no of uni-directional links between routers only
    _channels = _size * ( _h + _a -1) ; //each node's degree = global degree + local degree
    cout << "_channels: " << _channels << endl;
    
    cout << "_ComputeSize ends ..." << endl;
}
    
void DragonFlyFull::_BuildNet(const Configuration &config){
    cout << "inside _BuildNet() ..." << endl;
    
    int node, dst, count, channel_id, idx, width, channel_count, kk, link_type;
    ostringstream router_name; 
    
    all_routers.resize(_N);

    //create the routers
    for(node = 0; node < _N; node++){
        router_name.str("");
        router_name << "router";
        router_name << "_" << node;
        //cout << router_name.str() << endl;        
        _routers[node] = Router::NewRouter( config, this, router_name.str(), 
                    node, _radix, _radix );     //_radix's are the # of input and output ports, respectively
        _timed_modules.push_back(_routers[node]);
        all_routers[node] = _routers[node];
    }
    
    std::cout << "Routers created ..." << std::endl;
    
    //create the PEs
    //add input and output channels to processing nodes
    for(node = 0; node < _N; node++){
        for (count = 0; count < _p; count++ ) {
            channel_id = _p * node +  count;
                                    //if _p == 3:   
                                        //for router 0, channel_id is 0,1,2
                                        //for router 1, channel_id is 3,4,5 
                                        // and so on.
            _routers[node]->AddInputChannel( _inject[channel_id], _inject_cred[channel_id] );
                            // _inject and _inject_cred arrays are initialized in _alloc()
            _routers[node]->AddOutputChannel( _eject[channel_id], _eject_cred[channel_id] );
               
        }
    }
    
    std::cout << "PEs connected ..." << std::endl;
    
    
    //create the channels
    
    /*
    // g_graph already lists all the unidirectional links.
    // g_port_map contains the width of each link.
    // Based on these two, connect the actual links.
    
    // Go through each unidirectional link, check its width. 
    // Add that many outgoing channels in the src, and incoming channels in the dst.
    */
    channel_count = 0;
    for(node = 0; node < _N; node++){
        for(idx = 0; idx < (_h + _a - 1); idx++){
            dst = g_graph[node][idx];
            width = g_link_widths[node][idx];
            link_type = g_link_weights[node][idx];
            for (kk = 0; kk < width; kk++){
                //connect the channels
                _routers[node] -> AddOutputChannel(_chan[channel_count], _chan_cred[channel_count]);
                
                
                //Instead of controlling the latency through preprocessor derectives, control them with parameters.
                //Will be easier to control from testsripts.

                //#ifdef DRAGON_LATENCY
                
                if (link_type == LOCAL_LINK_WEIGHT){
	                _chan[channel_count]->SetLatency(_local_latency);
	                _chan_cred[channel_count]->SetLatency(_local_latency);
                }
                else if (link_type == GLOBAL_LINK_WEIGHT){
	                _chan[channel_count]->SetLatency(_global_latency);
	                _chan_cred[channel_count]->SetLatency(_global_latency);
                }
                
                //#endif

                
                _routers[dst] -> AddInputChannel(_chan[channel_count], _chan_cred[channel_count]);
                channel_count += 1;
            }
            //cout << "channel " << channel_count << " , node: " << node << " , dst: " << dst << " ,width: " << width << endl;
        
             
        }
    }
    

    //then go through the list and connect the links.
    cout << "done with _BuildNet() ..." << endl;
}
    
void DragonFlyFull :: _BuildGraphForLocal(){
    /*
    For each group, each node is connected with every other node in the group.
    The weight of each link is 1.
    */
    
    //global data types to be populated:
    //std::vector < std::vector <int> > g_graph;
    //std::vector < std::vector <int> > g_link_weights;
    
        int node;
        int neighbor;
        int node_group;
        int count;
    
    for(node = 0; node < _N; node++){
        //cout << "node " << node << endl;
        node_group = node/_a;
        //cout << "node group: " << node/_a << endl;
        
        //cout << "neighbors: ";
        for(neighbor = node_group * _a, count = 0; neighbor < node_group *_a + _a; neighbor++ ){
            
            if (neighbor != node){
                //cout << neighbor << " ";
            
                //add it to graph
                g_graph[node][count] = neighbor;
                g_link_weights[node][count] = LOCAL_LINK_WEIGHT;
                count += 1;
            }
            
        } 
        //cout << endl;
    }
    
    //    cout << "\nTest printing the graph:" << endl;    
    //    print_2d_vector(g_graph);
    //    cout << "\nTest printing the link weights:" << endl;
    //    print_2d_vector(g_link_weights);
    //    
}

void DragonFlyFull :: _BuildGraphForGlobal(string arrangement){
    /*
    Depending on the arrangement, we can follow different connection schemes.
    At the moment, we'll just support absolute_improved.
    */
    
    cout << "_BuilfGraphForGlobal() starts ..." << endl;
    
    //make a vector of links in the format of (src,dst) pairs
    std::vector< std::pair<int,int> > global_links; 
    
    
    if (arrangement == "absolute_improved"){
        std::vector <int> node_degree(_N, 0);   //a vector of length equal to total nodes, initialized to 0
        
        int src_node_id;
        int dst_node_id;
        
        int src_group;
        int src_node;
        
        int dest_group = 1;
        int dest_node = 0;
        
        for(src_group = 0; src_group < _g; src_group++){
            for (src_node = 0; src_node < _a; src_node++){
                while(node_degree[(_a * src_group) + src_node] < _h){
                
                    if ((dest_group != src_group) && (node_degree[(_a * dest_group) + dest_node] < _h)) {
                        src_node_id = (_a * src_group) + src_node;
                        dst_node_id = (_a * dest_group) + dest_node; 
                        
                        node_degree[src_node_id] += 1;
                        node_degree[dst_node_id] += 1;
                        
                        global_links.push_back(std::make_pair(src_node_id, dst_node_id));
                    }
                        
                    dest_group += 1;
                    
                    if (dest_group % _g == 0){
                        dest_node += 1;
                        dest_node %= _a;
                        dest_group %= _g;
                    }
                }
                
            }
        }
    }
    
    else{
        cout << "Error. Arrangement unsupported: " << arrangement << endl;
        exit(1);
    }
                
    //links connected. Test print.
    //    for(int ii=0; ii < global_links.size(); ii++){
    //        cout << "link " << ii << " : " << global_links[ii].first << " , " << global_links[ii].second << endl;
    //    }
    
    /*
        //We have the links. Now check if any link is present multiple times.
        //We need to assign multiple channels for that link.
        //For absolute_improved, this won't happen. But for some other arrangement 
        //scheme this can happen. So better to set up a system that can handle that.
        
        //Also, for a link (a,b), we need to check the weight of (b,a) as well,
        //and need to set the max of two as the weight of both the unidirectional links.
    */         
        
    //get frequency of the links
    for(std::size_t ii = 0; ii < global_links.size(); ii++){
        if ( g_global_link_frequency.find(global_links[ii]) != g_global_link_frequency.end() ){
            g_global_link_frequency[global_links[ii]] += 1;
        }else{
            g_global_link_frequency[global_links[ii]] = 1;
        }
    }
        
    //frequency counted. Test print.
    //    for(auto it = g_global_link_frequency.begin(); it!= g_global_link_frequency.end(); it++){
    //        cout << it->first.first << "," << it->first.second << " -> " << it->second << endl;
    //    }

    //go through the list again. For each (a,b), update the frequency of (a,b) and (b,a)
    //to its max. We do twice the amount of checking as actually needed, but its linear
    // order and done only once, so no big deal.
    int freq_a, freq_b, freq_max;
    for(std::size_t ii=0; ii < global_links.size(); ii++){
        if (g_global_link_frequency.find(global_links[ii]) != g_global_link_frequency.end()){
            freq_a = g_global_link_frequency[global_links[ii]];
        }else{
            freq_a = 0;
        }
    
        if (g_global_link_frequency.find( std::make_pair( global_links[ii].second, global_links[ii].first) ) != g_global_link_frequency.end()){
            freq_b = g_global_link_frequency[std::make_pair( global_links[ii].second, global_links[ii].first)];
        }else{
            freq_b = 0;
        }
    
        freq_max = freq_a > freq_b? freq_a : freq_b;
        
        g_global_link_frequency[global_links[ii]] = freq_max;
        g_global_link_frequency[std::make_pair( global_links[ii].second, global_links[ii].first)] = freq_max;
        
        //cout <<  global_links[ii].first << " , " << global_links[ii].second << " , " << freq_a << " , " << freq_b << " , " << freq_max << endl;
    }
            
    //    cout << "\n both way links: " << endl;
    //    for(auto it = g_global_link_frequency.begin(); it!= g_global_link_frequency.end(); it++){
    //        cout << it->first.first << "," << it->first.second << " -> " << it->second << endl;
    //    }
        
    //Links and their weights found. Now populate the graph accordingly.
    int src, dst, freq;
    int src_group, dst_group;
     int ii, jj;
    for(auto it = g_global_link_frequency.begin(); it != g_global_link_frequency.end(); it++){
        src = it->first.first;
        dst = it->first.second;
        freq = it->second;
        //cout << src << " , " << dst << " , " << freq << endl;
            
        //add it to src's neighbor list
        for(ii = 0; g_graph[src][ii] != -1; ii++);
        
        g_graph[src][ii] = dst;
        g_link_weights[src][ii] = GLOBAL_LINK_WEIGHT;
        
        //        cout << "graph[" << src << "]: ";
        //        for(ii = 0; ii < _h + _a - 1; ii++){
        //            cout << g_graph[src][ii] << " ";
        //        }
        //        cout << endl;

        //our links are unidirectional. So adding to the destination's neighbor list is not needed.
        
        //also, populate the g_inter_group_links vector.
        src_group = src/_a;
        dst_group = dst/_a;
        
        for(jj=0; jj < freq; jj++){
            g_inter_group_links[src_group][dst_group].push_back(std::make_pair(src,dst));
        }
    }
        
    /*cout << "\nTest printing the graph:" << endl;    
    print_2d_vector(g_graph);
    //    cout << "\nTest printing the link weights:" << endl;
    //    print_2d_vector(g_link_weights);
    //    
    cout << "test printing the g_inter_group_links ..." << endl; 
    for(int ii = 0; ii<_g; ii++){
        for(int jj = 0; jj<_g; jj++){
            cout << ii << "," << jj << " : ";
            for (int kk = 0; kk < g_inter_group_links[ii][jj].size(); kk++){
                cout << "(" << g_inter_group_links[ii][jj][kk].first << "," << g_inter_group_links[ii][jj][kk].second << ")" << " ";
            }
            cout << endl;
            
        }
        
    }*/

}

void DragonFlyFull :: _CreatePortMap(){
    /*
    Assumpiton: the graph is already generated. g_global_link_frequency is already populated.
    
    Each PE has a weight of 1.    
    Each local link has a width of 1. 
    Each global link has a width equal to its frequency.
    Populate the g_port_map accordingly.
    
    */
        
    //for each node
     int port_count;
    
    std::size_t node;
    int dst;
    
    int ii;
    int width;
    
    for (node = 0; node < g_graph.size(); node++){
        //first go through the local nodes
        port_count = _p;    //no fo PEs on eahc router
        
        for(ii = 0; ii < _a - 1; ii++ ){
            dst = g_graph[node][ii];
            g_link_widths[node][ii] = 1;    //by default, all local links has a width of 1
            g_port_map[std::make_pair(node,dst)] = std::make_pair(port_count, port_count);
            port_count += 1;
        }
    
        //now go through the global nodes
        for(ii = _a - 1 ; ii < _a - 1 + _h; ii++){
            dst = g_graph[node][ii];
            width = g_global_link_frequency[std::make_pair(node, dst)];
            
            g_link_widths[node][ii] = width;
            g_port_map[std::make_pair(node,dst)] = std::make_pair(port_count, port_count+width-1);
            
            port_count += width;
        }
    }
            
    //test print g_link_widths
    //cout << "\nTest printing the link widths:" << endl;
    //print_2d_vector(g_link_widths);
        
    //test print the g_port_map
    //    for (auto it = g_port_map.begin(); it != g_port_map.end(); it++){
    //            cout << "(" << it->first.first << "," << it->first.second << ") , ( " << it->second.first << "," << it->second.second << ")  ";  
    //            cout << endl;  
    //    }
}
    
    
void DragonFlyFull :: _generate_two_hop_neighbors(){
    /*
    This one will be needed for tiered routing.
    
    For a node, all node that are connected to it by two hops are the nodes:
        a) nodes in the groups connected to the source node by a global link 
        b) nodes connected by global links from the neighbor nodes in the same group as the source node.
        
    Logic:
        - Allocate the data structures
        - Go through all the global links (we have a data structure that stores them).
        - For each global link, get the src and dst 
            - For the src, list all the nodes in the dest's group as two-hop neighbors.
            - Add the src to the list of all dest_group_neighbors.
            - For the dst, list all the nodes in the src's group as the two-hop neighbors.
            - Add the dst to the list of all src_group_neighbors.
            
            
        - After the sets are populated, generate the vectors.
    */
    
    cout << "\ninside _generate_two_hop_neighbors()" << endl;
    
    //alocate the arrays.
    //std::vector < std::unordered_set <int> > two_hop_neighbors_set;
    //std::vector < std::vector <int> > two_hop_neighbors_vector;
    two_hop_neighbors_set.resize(_N, std::unordered_set<int>());
    two_hop_neighbors_vector.resize(_N, std::vector<int>());
    
    
    std::unordered_set<int>::iterator it;
    
    //here are all the global links
    //std::vector< std::vector < std::vector <std::pair<int,int> > > > g_inter_group_links;
    std::size_t ii, jj, kk;
    int src, dst;
    int src_group, dst_group;
    int src_neighbor, dst_neighbor;
    
    for(ii = 0; ii< g_inter_group_links.size(); ii++){
        for(jj = 0; jj < g_inter_group_links[ii].size(); jj++){
            //cout << "src: " << ii << "  dst: " << jj << " -> " << endl;
            for(kk = 0; kk < g_inter_group_links[ii][jj].size(); kk++){
                //cout << "(" << g_inter_group_links[ii][jj][kk].first << " , " << g_inter_group_links[ii][jj][kk].second << ")" << "    ";
                
                src = g_inter_group_links[ii][jj][kk].first;
                dst = g_inter_group_links[ii][jj][kk].second;
                
                src_group = src / _a;
                dst_group = dst / _a;
                
                //cout << "src: " << src << "  dst: " << dst << "  src_group: " << src_group << "  dst_group: " << dst_group << endl;
                
                //cout << "other nodes in src_group: " << endl;
                for(src_neighbor = src_group * _a + 0; src_neighbor < (src_group * _a + _a); src_neighbor++ ){
                    //cout << src_neighbor << " ";
                    
                    two_hop_neighbors_set[dst].insert(src_neighbor);
                    
                    two_hop_neighbors_set[src_neighbor].insert(dst);
                }
                //cout << endl;
                
                //cout << "other nodes in dst_group: " << endl;
                for(dst_neighbor = dst_group * _a + 0; dst_neighbor < (dst_group * _a + _a); dst_neighbor++ ){
                    //cout << dst_neighbor << " ";
                    two_hop_neighbors_set[src].insert(dst_neighbor);
                    two_hop_neighbors_set[dst_neighbor].insert(src);
                }
                //cout << endl;
                
            }
            //cout << endl;
                
        }
    }
    
    //now populate the vector 
    for(ii = 0; ii < _N; ii++){
        //cout << "node " << ii << " : " << "neighbors: " << two_hop_neighbors_set[ii].size() << " -> " ;
        for(it = two_hop_neighbors_set[ii].begin(); it != two_hop_neighbors_set[ii].end(); it++){
            //cout << *it << " ";
            two_hop_neighbors_vector[ii].push_back(*it);
        }
        //cout << endl;
    }
    
    //sort the vector
    for(ii = 0; ii < _N; ii++){
        std::sort(two_hop_neighbors_vector[ii].begin(), two_hop_neighbors_vector[ii].end());
    }
    
    //print the vector 
    //    cout << "\nafter sorting: " << endl;
    //    for(ii = 0; ii < _N; ii++){
    //        cout << "node " << ii << " : " << "neighbors: " << two_hop_neighbors_set[ii].size() << " -> " ;
    //        for(jj = 0; jj < two_hop_neighbors_vector[ii].size(); jj++){
    //            cout << two_hop_neighbors_vector[ii][jj] << " ";
    //        }
    //        cout << endl;
    //    }
    
    cout << "done with _generate_two_hop_neighbors()" << endl;

}



void DragonFlyFull :: _generate_one_hop_neighbors(){
    /*
    A list of one-global-hop neighbors for each node.
    
    Logic:
        Go through all the global links.
        For each src, add dst at its neighbor list.
    */
    
    //cout << "\ninside _generate_one_hop_neighbors()" << endl;
    
    std::size_t ii, jj, kk;
    int src, dst;
    std::unordered_set<int>::iterator it;
    
    //allocate data structures 
    one_hop_neighbors_set.resize(_N, std::unordered_set<int>() );
    one_hop_neighbors_vector.resize(_N, std::vector<int>() );
    
    //go through the links
    for(ii = 0; ii< g_inter_group_links.size(); ii++){
        for(jj = 0; jj < g_inter_group_links[ii].size(); jj++){
            //cout << "src_grp: " << ii << "  dst_grp: " << jj << " -> " << endl;
            for(kk = 0; kk < g_inter_group_links[ii][jj].size(); kk++){
                //cout << "(" << g_inter_group_links[ii][jj][kk].first << " , " << g_inter_group_links[ii][jj][kk].second << ")" << "    ";
                
                src = g_inter_group_links[ii][jj][kk].first;
                dst = g_inter_group_links[ii][jj][kk].second;
                //cout << src << " , " << dst << endl;
                
                one_hop_neighbors_set[src].insert(dst);
                one_hop_neighbors_set[dst].insert(src);
            }
        }
    }
    
    //now generate vectors from unordered_set
    for (ii = 0; ii < _N; ii++ ){
        for (it = one_hop_neighbors_set[ii].begin(); it != one_hop_neighbors_set[ii].end(); it++){
            one_hop_neighbors_vector[ii].push_back(*it);
        }
    }
    
    //now sort the vectors
    for (ii = 0; ii < _N; ii++){
        std::sort(one_hop_neighbors_vector[ii].begin(), one_hop_neighbors_vector[ii].end());
    }
    //print the vector 
    /*for(ii = 0; ii < _N; ii++){
        cout << "node " << ii << " : " << "neighbors: " << one_hop_neighbors_set[ii].size() << " -> " ;
        for(jj = 0; jj < one_hop_neighbors_vector[ii].size(); jj++){
            cout << one_hop_neighbors_vector[ii][jj] << " ";
        }
        cout << endl;
    }*/
    
    
    
    cout << "done with _generate_one_hop_neighbors()" << endl;

}

void DragonFlyFull :: _generate_common_neighbors_for_group_pair(){
    // for each group pair, it lists the nodes that has global links to 
    // each of these groups.
    // Say, node 0 has global links to groups 2,3,4 and 5.
    // Then group pairs (2,3), (2,4), (2,5), (3,4), (3,5) and (4,5) all will
    // list 0 as one such node.

    // So we need to generate all the possible combinations of length two.
    // use conbinations() in combinations.cpp.
    

    //cout << "inside _generate_common_neighbor_nodes_for_group_pair()" << endl;

    int node;
    int ii;
    int neighbor;
    int start, end, len; 
    int src_node, dst_node, src_group, dst_group;

    //cache for combination generation funtion. We'll probably call combination
    //with a single pair of arguments for a single execution, so it makes sense
    //to cache the value.
    //pair<int,int> vs vector<vector<int>>
    std::unordered_map < std::pair<int, int>, std::vector <std::vector <int> >, pair_hash > combination_cash;

    std::pair<int,int> group_pair;
    std::vector< std::vector<int> > combos;

    //go through the graph
    for(node = 0; node < _N; node ++){
        
        //cout << "node: " << node << endl; 

        start = _a - 1;
        end = g_graph[node].size();
        len = end - start;

        //for combinations we need cash of type pair<int,int> vs vector<vector<int>>
        
        if (combination_cash.find( std::make_pair(len,2) ) != combination_cash.end() ){
            //cout << "cash hit with " << len << ",2" << endl;
            combos = combination_cash[std::make_pair(len,2)];
        }else{
            combinations(len, 2, combos);
            //cout << "cash miss for " << len << ",2. Adding." << endl; 
            combination_cash[std::make_pair(len,2)] = combos;
        }
        
        //generate group pair for each combo
        for (ii = 0; ii < combos.size(); ii++){
            src_node = g_graph[node][start + combos[ii][0]];
            dst_node = g_graph[node][start + combos[ii][1]];
            src_group = src_node / _a;
            dst_group = dst_node / _a;

            group_pair = std::make_pair(src_group, dst_group);

            if (g_group_pair_vs_common_nodes.find(group_pair) != g_group_pair_vs_common_nodes.end()){
                g_group_pair_vs_common_nodes[group_pair].push_back(node);   
            }else{
                g_group_pair_vs_common_nodes[group_pair] = {node};
            }

            group_pair = std::make_pair(dst_group, src_group);

            if (g_group_pair_vs_common_nodes.find(group_pair) != g_group_pair_vs_common_nodes.end()){
                g_group_pair_vs_common_nodes[group_pair].push_back(node);   
            }else{
                g_group_pair_vs_common_nodes[group_pair] = {node};
            }            
        }
    }

    // cout << "g_group_pair_vs_common_nodes contents: " << endl;
    
    // for (auto it = g_group_pair_vs_common_nodes.begin(); it != g_group_pair_vs_common_nodes.end(); it++){
    //     cout << it->first.first << "," << it->first.second << " : ";
    //     for(ii = 0 ; ii < it->second.size(); ii++){
    //         cout << it->second[ii] << " ";
    //     }
    //     cout << endl;
    // }

    // cout << "leaving _generate_common_neighbor_nodes_for_group_pair()" << endl;
}


    
    
void DragonFlyFull :: RegisterRoutingFunctions(){
    cout << "inside _RegisterRoutingFunctions() ..." << endl;

    gRoutingFunctionMap["min_dragonflyfull"] = &min_dragonflyfull;
    gRoutingFunctionMap["min_djkstra_dragonflyfull"] = &min_djkstra_dragonflyfull;
    
    gRoutingFunctionMap["vlb_dragonflyfull"] = &vlb_dragonflyfull;
    gRoutingFunctionMap["vlb_restricted_src_only_dragonflyfull"] = &vlb_dragonflyfull;
    gRoutingFunctionMap["vlb_restricted_src_and_dst_dragonflyfull"] = &vlb_dragonflyfull;
    
                    //Variation of different versions of VLB (regular and
                    // restricted) is done
                    // through the same mother VLB function. Internally, path 
                    // selection options are toggled by the global switch
                    // g_routing_mode. The value of the switch is set in the
                    //the function _setRoutingMode().
    gRoutingFunctionMap["vlb_four_hop_restricted_dragonflyfull"] = &vlb_dragonflyfull;
                    //This is even more restricted than vlb_restricted.
                    //The maximum allowed length for vlb paths are four hops.
    gRoutingFunctionMap["vlb_four_hop_some_five_hop_restricted_dragonflyfull"] = &vlb_dragonflyfull;
    


    gRoutingFunctionMap["UGAL_L_dragonflyfull"] = &UGAL_dragonflyfull;
    
    gRoutingFunctionMap["UGAL_L_two_hop_dragonflyfull"] = &UGAL_dragonflyfull;
                    //Variation of different versions of UGAL (L and G) is done
                    //through the same mother UGAL function. Internally, path 
                    // selection options are toggled by the global switch
                    // g_routing_mode. The value of the switch is set in the
                    //the function _setRoutingMode().

    gRoutingFunctionMap["UGAL_L_restricted_src_only_dragonflyfull"] = &UGAL_dragonflyfull;
    gRoutingFunctionMap["UGAL_L_restricted_src_and_dst_dragonflyfull"] = &UGAL_dragonflyfull;
        //Both restricted and two_hop use intermediate nodes that lead to 
        //shorter paths. 
        //Difference is, two_hop uses min_df paths, and restricted uses
        //djkstra paths.
    gRoutingFunctionMap["UGAL_L_four_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;
    
    gRoutingFunctionMap["UGAL_L_three_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;
        

    gRoutingFunctionMap["UGAL_L_threshold_dragonflyfull"] = &UGAL_dragonflyfull;
    
    gRoutingFunctionMap["UGAL_L_four_hop_some_five_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;
    
    gRoutingFunctionMap["UGAL_G_dragonflyfull"] = &UGAL_dragonflyfull;
                    //UGAL_G and Ugal_L both implemented through the same UGAL function.
                    //The difference is only in the final min vs vlb path choice decision,
                    //which is controlled by a global switch g_ugal_local_vs_global_switch.
                    //The value of the switch is set in the function _setRoutingMode().
    gRoutingFunctionMap["UGAL_G_restricted_src_only_dragonflyfull"] = &UGAL_dragonflyfull;
    gRoutingFunctionMap["UGAL_G_restricted_src_and_dst_dragonflyfull"] = &UGAL_dragonflyfull;

    gRoutingFunctionMap["UGAL_G_four_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;
    gRoutingFunctionMap["UGAL_G_three_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;
    

    gRoutingFunctionMap["UGAL_G_four_hop_some_five_hop_restricted_dragonflyfull"] = &UGAL_dragonflyfull;


    gRoutingFunctionMap["PAR_dragonflyfull"] = &PAR_dragonflyfull;
    gRoutingFunctionMap["PAR_restricted_src_only_dragonflyfull"] = &PAR_dragonflyfull;
    gRoutingFunctionMap["PAR_restricted_src_and_dst_dragonflyfull"] = &PAR_dragonflyfull;
    gRoutingFunctionMap["PAR_four_hop_restricted_dragonflyfull"] = &PAR_dragonflyfull;
    gRoutingFunctionMap["PAR_three_hop_restricted_dragonflyfull"] = &PAR_dragonflyfull;
    gRoutingFunctionMap["PAR_four_hop_some_five_hop_restricted_dragonflyfull"] = &PAR_dragonflyfull;
    
    
    cout << "done with _RegisterRoutingFunctions() ..." << endl;
}

void min_djkstra_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject){
    /*
    We already have the djkstra parents table formulated, so we can generate all the paths
    between a particular src and dst on the fly.
    
    The routing function needs to find:
        - an output port
        - an output vc
        
    For the output port:
        - First check if it is an injection port. If yes, do accordingly.
        - Then check if it is the destination router. If yes, then find the port
         to the destination PE.
        - Then check if it is the source router. If yes:
            - Call path-generator function to generate the complete path from the src and
            dest router using djkstra table.
            - Save the whole path in the flit (TODO: modification in the Flit class
            necessary to accomodate the path and hop-count within the flit).
        - Based on the hop count, find the port id to the next router.
        - Increase hop-count.
        
    For the output vc:
        - give a new vc on each hop.
            - For non-minimal routing, max hop-count is 6. So we need 6 + 1(for injection) vcs.
        - For flit injection, assign to vc 0. 
            - Will that lead to injection vs congestion? Check.
            - In that case, we can just randomly assign a vc. Probably not harmful.
    
    So two extra functions needed:
        - A function to generate the complete path for the flit, give a pair of src and dst.
        - A function for find the port to the next hop given the flit's hop count, 
        saved path, current and next routers.
    */
    
    bool flag = false;

    //First check if it is an injection port. If yes, do accordingly.
    
    if(inject) {
        outputs->Clear( ); //doesn't matter really. If the flit is at injection,
                            //means it was just generated, so the outputset is empty anyway.

        int inject_vc= RandomInt(gNumVCs-1);
        outputs->AddRange(-1, inject_vc, inject_vc);
          
        //if((f->watch) && ((f->id == 0) || (f->id == 100) ) ) {
        /*if(((f->id == 0) || (f->id == 100) ) ) {
            cout << "inside inject block for flit:" << f->id << endl;
        }*/

        g_total_flit += 1;

    return;
    }
    
    //gather necessary info from the flit
    outputs -> Clear();
    
    int current_router = r->GetID();
    
    int out_port = -33;    //instead of setting all the default error values to -1, we used different values in different places. Works kinda like different error codes, helps in debugging.
    
    int out_vc = 0;
    
    int dst_PE = f->dest;
    int dst_router = dst_PE / g_p;
    int selected_min_path_hop_count;

    //Then check if it is the destination router. 
    //If yes, then find the port to the destination PE.
    if(current_router == dst_router){
        out_vc = RandomInt(gNumVCs-1);
            //This could be a static VC as well. But a PE is basically a exit drain for packets,
            //so differentiating between VCs doesn't seem neceesary. Rather just draining them as fast as possible.
        
        //find the port that goes to the destination processing node
        out_port = dst_PE % g_p;
        
    }
    else if (f->hop_count == 0){   //source router
        //Call path-generator function to generate the complete path from the src and
        //    dest router using djkstra table.
        std::vector<int> pathVector;
        
        //cout << "calling select_shortest_path_djkstra() for src, dst pair: " << current_router << "," << dst_router << endl;
        int temp = select_shortest_path_djkstra(current_router, dst_router, pathVector);
        
        if (flag){
            cout << "value from select_shortest_path_djkstra() returned: " << temp << endl;
            cout << "the path returned is: ";
            for(std::size_t ii = 0; ii < pathVector.size(); ii++){
                cout << pathVector[ii] << " ";
            }
            cout << endl;
        }

        //Save the whole path in the flit
        f->path = std::move(pathVector);
        
        //now get the port to the next hop node 
        out_port = find_port_to_node(current_router, f->path[1]);
        
        //assign vc. first hop, so vc 0.
        out_vc = 0;
        
        //increase hop_count
        f->hop_count += 1;

        //collect path stat
        g_total_min_flit += 1;
        
        selected_min_path_hop_count = f->path.size() - 1;

        if (dst_router/g_a == current_router/g_a){
            g_min_ingroup_path_dist[selected_min_path_hop_count] += 1;
        }else{
            g_min_outgroup_path_dist[selected_min_path_hop_count] += 1;
        }

    
    }else{  //neither src nor dst router. Packet in flight.
        //get current hop count 
        
        //get port to the next hop node
        out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
        
        //assign vc 
        out_vc = f->hop_count;
        
        //increase hop_count
        f->hop_count += 1;
        
        
    }

    //finally, build the output set
    outputs->AddRange( out_port, out_vc, out_vc );
    
}

/*
For routing:
- We need to populate the djkstra parents table. That requires:
    - Making the adjacency list
    - Declaring the parents and weighs data strucures.
    - Call djkstra

- This needs to be done in the constructor, after the network is built.
    
- Finally, use the parents data structure to generate paths between every S-D 
pair as required.
*/

void DragonFlyFull :: _discover_djkstra_paths(){
    //go through the graph
    
    //for every node, add all it's neighbors and their weights as adjacency list.
    
    //adjacency_list can be local. parents (and weights, though we dont use it 
    //anywhere now) needs to be global so that the routing functions can access them.
    
    cout << "\ninside _discover_djkstra_paths()" << endl;
    
    std::vector < std::vector < std::pair<int,int> > > weighted_adjacency_list(_N);
    int dst, weight;
    
    for( int src = 0; src < _N; src++){
        weighted_adjacency_list[src].resize(g_graph[src].size());
    }    
    
    for( int src = 0; src < _N; src++){
        for( std::size_t jj = 0; jj < g_graph[src].size(); jj++){
            dst = (int)g_graph[src][jj];
            weight = (int)g_link_weights[src][jj];
            //cout << "(src,dst,weight) : " << src << ", " << dst << ", " << weight << endl;
            weighted_adjacency_list[src][jj] = std::make_pair(dst, weight);
        }
    }
    
    //    cout << "weighted_adjacency_list generated. Test printing:" << endl;
    //    for( int src = 0; src < _N; src ++){
    //        cout << src << " : ";
    //        for( int jj = 0; jj < weighted_adjacency_list[src].size(); jj++){
    //            cout << "(" << weighted_adjacency_list[src][jj].first << ", " << weighted_adjacency_list[src][jj].second << " ) ,";
    //        }
    //        cout << endl;
    //    
    //    }
    
    //allocate arrays
    g_distance.resize(_N, std::vector< int >(_N,INF));
    g_parents.resize(_N, std::vector< std::vector<int> >(_N));
    cout << "g_distance and g_parents allocated." << endl;

    //now call djkstra
    all_pair_djkstra(_N, weighted_adjacency_list, g_distance, g_parents);
    cout << "all_pair_djkstra() returned." << endl;
    
}

int select_shortest_path_djkstra(int src_router, int dst_router, std::vector<int> & pathVector){
    /*
    For a particular src and dst router pair, generate all the djkstra minimal paths.
    If there are more than one, randomly select one. 
    Copy the selected path in pathVector.
    Return 1 if path found, -1 if not. 
    */

    bool flag = false;

    //need to pass the flit in definition if we really need to track here.
    /*if (f->id == FLIT_TO_TRACK){
        flag = true;
    }*/

    if(flag){
        cout << "inside select_shortest_path_djkstra() " <<  endl;
        cout << "src:" << src_router << endl;
        cout << "dst:" << dst_router << endl;
    }
    
    int selected_path_id;
    int path_count;
    
    std::vector< std::vector<int> > final_paths;
    
    generate_path(src_router, dst_router, g_parents, final_paths);
    
    path_count = final_paths.size();
    
    if (path_count == 0){
        cout << "no paths found between router pairs " << src_router << " and " << dst_router << endl; 
        return -1;
    }else{
        selected_path_id = RandomInt(path_count-1); //RandomInt gets a number in the range[0,max], inclusive.
        pathVector = std::move(final_paths[selected_path_id]); 
        return 1;
    }
    
}

int find_port_to_node(int current_router, int next_router){
    /*
    As the name suggests. Get the port and return.
    
    Each router first have _p number of PEs connected. Jump over them.
    Then go through the graph. Check each neighbor, count how many channels go to it.
    Stop when the next_router is found.
    If more than one channel goes to next_router, randomly select one.
    
    Return -1 if no port found.
    */
    
    //we already populated g_port_map for this purpose
    
    auto port_range = g_port_map[std::make_pair(current_router, next_router)];
    
    int selected = RandomInt(port_range.second - port_range.first); 
                    //range returned by g_port_map is a closed range. For example, for port 5 it'll return (5,5) pair
                    //RandomInt also returns an int in the range [0, x]. So subtracting 1 is not needed in this case.
    
    return (port_range.first + selected);
            //There was an error here. Instead of first, I put second and 
            //added "selected" to it. However, this didnt throw any error
            //as none of the dragonfly links we used had more than 1 channel.
            //So, first and second were always same and could be used //interchangebly. The error only became apparent in Edison.
            //Here updating it without rigorous testing, so leaving a breadcrumb.  
    
}

void min_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject){
    /*
    Djkstra gives the absolute minimum paths and that could be a problem. 
    For local traffic, we do not want all the traffic to go through a one-hop
    or two-hop paths when some more three-hop paths (with only one global link) are
    available (as the latency of the global link dominates routing performance).
    
    So need to calculate paths more traditionally. 
    
    The routing function needs to find:
        - an output port
        - an output vc
        
    For the output port:
        - First check if it is an injection port. If yes, do accordingly.
        
        - Then check if it is the destination router. If yes, then find the port
         to the destination PE.
         
        - Then check if it is the source router. If yes:
            - Get destination group.
            - Get a list of nodes in current group that goes directly to the destination group.                
                [g_inter_group_links maintains this list. Just look it up.]
            - Pick one randomly.
            - Then have a path from src_router -> gateway_router -> gateway_router_at_dst_group -> dst_router.  
            
            - Save the whole path in the flit 
        
        - Based on the hop count, find the port id to the next router.
        - Increase hop-count.
        
    For the output vc:
        - give a new vc on each hop.
            - For non-minimal routing, max hop-count is 6. So we need 6 + 1(for injection) vcs.
        - For flit injection, assign to vc 0. 
            - Will that lead to injection vs congestion? Check.
            - In that case, we can just randomly assign a vc. Probably not harmful.
    
    So two extra functions needed:
        - A function to generate the complete path for the flit, given a pair of src and dst.
        - A function for finding the port to the next hop given the flit's hop count, 
        saved path, current and next routers.
    */
    
    //First check if it is an injection port. If yes, do accordingly.
    
    bool flag = false;

    if(inject) {
        outputs->Clear( ); //doesn't matter really. If the flit is at injection,
                            //means it was just generated, so the outputset is empty anyway.

        int inject_vc= RandomInt(gNumVCs-1);
        outputs->AddRange(-1, inject_vc, inject_vc);
          
        //if((f->watch) && ((f->id == 0) || (f->id == 100) ) ) {
        /*if(((f->id == 0) || (f->id == 100) ) ) {
            cout << "inside inject block for flit:" << f->id << endl;
        }*/

        g_total_flit += 1;

        return;
    }
    
    //gather necessary info from the flit
    outputs -> Clear();
    
    int current_router = r->GetID();
    int prev_router;
    int next_router;
    
    int out_port = -33;    //instead of setting all the default error values to -1, we used different values in different places. Works kinda like different error codes, helps in debugging.
    
    int out_vc = 0;
    
    int dst_PE = f->dest;
    int dst_router = dst_PE / g_p;
    int selected_min_path_hop_count;
    
    //Then check if it is the destination router. 
    //If yes, then find the port to the destination PE.
    if(current_router == dst_router){
        out_vc = RandomInt(gNumVCs-1);
            //This could be a static VC as well. But a PE is basically a exit drain for packets,
            //so differentiating between VCs doesn't seem neceesary. Rather just draining them as fast as possible.
        
        //find the port that goes to the destination processing node
        out_port = dst_PE % g_p;
        
    }
    else if (f->hop_count == 0){   //source router
    
        std::vector<int> pathVector;
        
        //cout << "calling select_shortest_path() for src, dst pair: " << current_router << "," << dst_router << endl;
        
        int temp = select_shortest_path(current_router, dst_router, pathVector);
        
        if (flag){
            cout << "value from select_shortest_path() returned: " << temp << endl;
            cout << "the path returned is: ";
            for(std::size_t ii = 0; ii < pathVector.size(); ii++){
                cout << pathVector[ii] << " ";
            }
            cout << endl;
        }

        //Save the whole path in the flit
        f->path = std::move(pathVector);
        
        //now get the port to the next hop node 
        out_port = find_port_to_node(current_router, f->path[1]);
        
        //assign vc. first hop, so vc 0.
        out_vc = 0;
        
        //increase hop_count
        f->hop_count += 1;

        //collect path stat
        g_total_min_flit += 1;
        
        selected_min_path_hop_count = f->path.size() - 1;

        if (dst_router/g_a == current_router/g_a){
            g_min_ingroup_path_dist[selected_min_path_hop_count] += 1;
        }else{
            g_min_outgroup_path_dist[selected_min_path_hop_count] += 1;
        }



        if (g_log_Qlen_data == 1){
                string q_len_data = "";                
                int min_q_len;

                min_q_len = find_port_queue_len_to_node(r, f->path[0], f->path[1]);

                //cout << "Router:, " << r->GetID() << " , " << "min_q:, " << min_shortest_path_weight << " , " << "non-min_q:, " << min_VLB_path_weight << " , chosen:, " << chosen << endl;
            //return the id of the chosen one
                q_len_data += std::to_string(r->GetID());
                q_len_data += ",";
                q_len_data += std::to_string(min_q_len);
                q_len_data += "\n";
                
                //cout << q_len_data;
                ed_q_len_file << q_len_data;
            
        }

    
    }else{  //neither src nor dst router. Packet in flight.
        //get current hop count 
        
        //get port to the next hop node
        out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
        
        //assign vc 
        if (g_vc_allocation_mode == "incremental"){
            out_vc = f->hop_count;
        }
        else{
            prev_router = f->path[f->hop_count - 1];
            next_router =  f->path[f->hop_count + 1];
            out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
        }

        //increase hop_count
        f->hop_count += 1;
        
    }

    //finally, build the output set
    outputs->AddRange( out_port, out_vc, out_vc );
    
}


int select_shortest_path(int src_router, int dst_router, std::vector<int> & pathVector){
    /*
    Get the src and dest group.
    Get destination group.
            - Get a list of nodes in current group that goes directly to the destination group. [g_inter_group_links maintains this list. Just look it up.]
            - Pick one randomly.
            - Then have a path from src_router -> gateway_router -> gateway_router_at_dst_group -> dst_router.  
            
    Copy the selected path in pathVector.
    Return 1 if path found, -1 if not. 
    */
    

    int src_group, dst_group;
    int link_to_select;
    std::pair<int,int> selected_global_link;
    
    src_group = (int)src_router / g_a;
    dst_group = (int)dst_router / g_a;
            
    //case 0: src and dst are in the same group.
    if (src_group == dst_group){
        pathVector = {src_router, dst_router};
    }
    
    //case 1: src and dst are in different groups
    else{
        if (g_inter_group_links[src_group][dst_group].size() == 0){
            cout << "no paths found between router pairs " << src_router << " and " << dst_router << endl; 
            return -1;
        }
        
        link_to_select = RandomInt(g_inter_group_links[src_group][dst_group].size() - 1); 
        selected_global_link = g_inter_group_links[src_group][dst_group][link_to_select];
        
        if ((src_router == selected_global_link.first) && (dst_router == selected_global_link.second)){
            pathVector = {src_router, dst_router};
        }
        else if (src_router == selected_global_link.first){
            pathVector = {src_router, selected_global_link.second, dst_router};
        }
        else if (dst_router == selected_global_link.second){
            pathVector = {src_router, selected_global_link.first, dst_router};
        }
        else{
            pathVector = {src_router, selected_global_link.first, selected_global_link.second, dst_router};
        }
    }
    return 1;

}

void vlb_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject){
    /*
    Regular VLB routing function.
    For a Src and Dst, (randomly) select an Intermediate node.
    Then route (src, imdt) and (imdt, dst).
    
    Outsource the imdt-node selection to a eparate function so that it can be manipulated later, if needed.
    
    As usual,
    The routing function needs to find:
        - an output port
        - an output vc
        
    For the output port:
        - First check if it is an injection port. If yes, do accordingly.
        
        - Then check if it is the destination router. If yes, then find the port
         to the destination PE.
         
        - Then check if it is the source router. If yes:
            - if src and dst are in the same group, route them normally.
            
            - else, select an intermediate node. 
            
            - then generate paths in the form of (min, imdt, dst)
            
            - Save the whole path in the flit 
            
        - Based on the hop count, find the port id to the next router.
        - Increase hop-count.
        
    For the output vc:
        - give a new vc on each hop.
            - For non-minimal routing, max hop-count is 6. So we need 6 + 1(for injection) vcs.
        - For flit injection, assign to vc 0. 
            - Will that lead to injection vs congestion? Check.
            - In that case, we can just randomly assign a vc. Probably not harmful.
    
    So two functions needed:
        - A function to generate the complete path for the flit, given a pair of src and dst.
        - A function for finding the port to the next hop given the flit's hop count, 
        saved path, current and next routers.
    */
    
    //First check if it is an injection port. If yes, do accordingly.
    
    bool flag = false;

    if(inject) {
        outputs->Clear( ); //doesn't matter really. If the flit is at injection,
                            //means it was just generated, so the outputset is empty anyway.

        int inject_vc= RandomInt(gNumVCs-1);
        outputs->AddRange(-1, inject_vc, inject_vc);
          
        return;
    }
        
    //gather necessary info from the flit
    outputs -> Clear();
    
    int current_router = r->GetID();
    int prev_router;
    int next_router;

    int out_port = -33;    //instead of setting all the default error values to -1, we used different values in different places. Works kinda like different error codes, helps in debugging.
    
    int out_vc = 0;
    
    int dst_PE = f->dest;
    int dst_router = dst_PE / g_p;
    
    int selected_vlb_path_hop_count;

    //Then check if it is the destination router. 
    //If yes, then find the port to the destination PE.
    if(current_router == dst_router){
        out_vc = RandomInt(gNumVCs-1);
            //This could be a static VC as well. But a PE is basically a exit drain for packets,
            //so differentiating between VCs doesn't seem neceesary. Rather just draining them as fast as possible.
        
        //find the port that goes to the destination processing node
        out_port = dst_PE % g_p;

        g_total_flit += 1;
        
    }
    else if (f->hop_count == 0){   //source router
        //actual VLB routing needs to happen here.
        
        std::vector<int> pathVector;
        
        //cout << "calling select_vlb_path() for src, dst pair: " << current_router << "," << dst_router << endl;
        
        //int temp = select_vlb_path_regular(f, current_router, dst_router, pathVector);
        int temp = select_vlb_path(f, current_router, dst_router, pathVector);
                //this function supports both regular and restricted vlb modes.

        if (flag){
            cout << "value from select_vlb_path() returned: " << temp << endl;
            cout << "the path returned is: ";
            for(std::size_t ii = 0; ii < pathVector.size(); ii++){
               cout << pathVector[ii] << " ";
            }
            cout << " : pathlen " << pathVector.size() -1;
            cout << endl;       
        }

        //Save the whole path in the flit
        f->path = std::move(pathVector);
        
        //now get the port to the next hop node 
        out_port = find_port_to_node(current_router, f->path[1]);
        
        //assign vc. first hop, so vc 0.
        out_vc = 0;
        
        //increase hop_count
        f->hop_count += 1;

        //collect path stat
        g_total_non_min_flit += 1;
        
        selected_vlb_path_hop_count = f->path.size() - 1;

        if (dst_router/g_a == current_router/g_a){
            g_vlb_ingroup_path_dist[selected_vlb_path_hop_count] += 1;
        }else{
            g_vlb_outgroup_path_dist[selected_vlb_path_hop_count] += 1;
        }

    
    }else{  //neither src nor dst router. Packet in flight.
        //get current hop count 
        
        //get port to the next hop node
        out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
        
        //assign vc 
        if (g_vc_allocation_mode == "incremental"){
            out_vc = f->hop_count;
        }
        else{
            prev_router = f->path[f->hop_count - 1];
            next_router =  f->path[f->hop_count + 1];
            out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
        }

        //increase hop_count
        f->hop_count += 1;
        
    }

    //finally, build the output set
    outputs->AddRange( out_port, out_vc, out_vc );
    
}


/*
    Update (7 sept 2018): 
    
    This is wrong. The intermediate path selection logic needs to changed.    
    
    So what do we need?
        
        - Just select any other node from another group.
        - then call the select_shortest_path() to get (src,imdt) and (imdt, dst) paths.
         
    That seems trivial. How does pruning happen?
        - One option: if we already know the list of imdt links in each tier, we can select from there.
        - Option two: 
            If we only want two hop paths, we only want to get to the nodes that are connected directly 
            with the nodes in this group. 
            We can get that list by looking up in g_inter_group_links for src_group[] and all other dest groups.
            May be we can do a preprocessing and generate a list for each src group in the constructor.
    
        Implement these. Then move to UGAL.
            //in UGAL, consider vc ocupancy and how that can affect the performance while path selection.
        
    */
        
int select_vlb_path_old(int src_router, int dst_router, std::vector<int> & pathVector){
    /*
    First check if the src_router and dst_router are in the same group. If yes, then route minimally.
    
    Else,
        Choose an imdt_router. 
            - This is where some pruning can be made.
        
    Then generate path with (src_router, imdt, dst_router).
        #path generation will also depend on how imdt_router is chosen.
        #so the node-choosing and path-generation might be in the same place.
        
    Copy the selected path in pathVector.
    Return 1 if path found, -1 if not. 
    */
    
    //cout << "inside select_vlb_path() for src_router " << src_router << " dst_router " << dst_router << endl;
    
    bool flag = false;

    int src_group, dst_group;
    std::pair<int,int> selected_global_link;
    
    int ii, jj;
    std::vector<int> gateway_router_list;
    int chosen_gateway;
    int selected_group;
    int imdt_router;
    
    src_group = src_router / g_a;
    dst_group = dst_router / g_a;
    
    //case 0: src and dst are in the same group.
    if (src_group == dst_group){
        pathVector = {src_router, dst_router};
    }
    
    //case 1: src and dst are in different groups
    else{
        
        //find the groups directly connected with the source
            //the graph contains links for each node. ignore the local links, just check the global links.
            //for each global link, get the group it leads to.
            //maintain a list of groups conencted to this src_router.
            //now imdt_node selection: 
                //First, randomly choose one group from this list.
                //Then, randomly choose a node from the chosen group.
            //This way, the group with more global_link to will have more selection probability,
                //which seems logical.
            
        std::vector<int> first_half_pathVector;
        
        //cout << "global neighbors of " << src_router << " : ";
        for(ii = g_a-1; ii < g_graph[src_router].size(); ii++){ //loop starts from (g_a-1) to avoid local neighbors
            //cout << g_graph[src_router][ii] << " " << (int)g_graph[src_router][ii]/g_a << endl;
            for (jj=0; jj < g_link_widths[src_router][ii]; jj++){
                gateway_router_list.push_back( (int)g_graph[src_router][ii] );        
            }
        }
        
        //        cout << "gateway routers: " ; 
        //        for(ii=0; ii< gateway_router_list.size(); ii++){
        //            cout << gateway_router_list[ii] << " ";
        //        }
        //        cout << endl;
        //        
        chosen_gateway = gateway_router_list[ RandomInt(gateway_router_list.size() - 1) ];
        //cout << "chosen_gateway: " << chosen_gateway << endl;
        
        selected_group = chosen_gateway / g_a;
        //cout << "selected_group: " << selected_group << endl;
        
        imdt_router = (selected_group * g_a) + RandomInt(g_a-1);
        //cout << "imdt_router: " << imdt_router << endl; 
                 
        //generate route to the selected node
        first_half_pathVector.push_back(src_router);
        first_half_pathVector.push_back(chosen_gateway);
        if (imdt_router != chosen_gateway){
            first_half_pathVector.push_back(imdt_router);
        } 
        
        //        cout << "first half of the path: ";
        //        for (ii = 0; ii < first_half_pathVector.size(); ii++){
        //            cout << first_half_pathVector[ii] << " ";
        //        }
        //        cout << endl;
        //        
        
        //first half of the path is formed. Now generate the imdt->dest block.
        //For this, unmodified min-routing is being used. 
        //So we can just call the select_shortest_path() function
        
        std::vector<int> second_half_pathVector;
        
        //cout << "calling select_shortest_path() for src, dst pair: " << current_router << "," << dst_router << endl;
        
        select_shortest_path(imdt_router, dst_router, second_half_pathVector);
        
        if (flag){
               cout << "second half of the path: ";
               for (ii = 0; ii < second_half_pathVector.size(); ii++){
                   cout << second_half_pathVector[ii] << " ";
               }
               cout << endl;      
        }

        //join the two vectors
        pathVector.reserve(first_half_pathVector.size() + second_half_pathVector.size());
        pathVector.insert(pathVector.end(), first_half_pathVector.begin(), first_half_pathVector.end());
        pathVector.insert(pathVector.end(), second_half_pathVector.begin() + 1, second_half_pathVector.end());
        
        //        cout << "joined path: ";
        //        for (ii = 0; ii < pathVector.size(); ii++){
        //            cout << pathVector[ii] << " ";
        //        }
        //        cout << endl;
        //        
    }
    return 1;

}


int select_vlb_path_regular(const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector){
    /*
        Plain vanilla VLB routing.
        
        Randomly select an intermediate node not within the group.
        
        Get paths from src to imdt, and then from imdt to dst.
        
        Join the paths. Win. 
        
        Returns:    The selected path in pathVector.
                    
                    The selected intermediate path used to generate the path. 
                    This helps to avoid duplicate paths in UGAL routing where multiple VLB paths are considered.
        
        
        The flit is passed for debugging.
        */    
        
    //cout << "inside select_vlb_path_regular for " << "src: " << src_router << " dest: " << dst_router << "  " << endl;
    
    int src_group, dst_group;
    std::pair<int,int> selected_global_link;
    
    std::vector<int> gateway_router_list;
    int imdt_router;

    src_group = src_router / g_a;
    dst_group = dst_router / g_a;
        
    //cout << "src_group: " << src_group << " dst_group: " << dst_group << endl; 
    
    //case 0: src and dst are in the same group.
    //TODO: ship this out to it's own function.
    if (src_group == dst_group){
        //pathVector = {src_router, dst_router};
        
        imdt_router = select_vlb_path_inside_group(src_router, dst_router, pathVector);
        
        //        cout << "same group. path: ";
        //        for(ii = 0; ii < pathVector.size(); ii++){
        //            cout << pathVector[ii] << " ";
        //        }
        //        cout << endl;
    }
    
    //case 1: src and dst are in differnet groups 
    else{
        std::vector<int> first_half_pathVector;
        std::vector<int> second_half_pathVector;
        
        //select intermediate node 
        //shipping it out to a separate function for easy modifications.
        imdt_router = vlb_intermediate_node_vanilla(f, src_router, dst_router);
        
        select_shortest_path(src_router, imdt_router, first_half_pathVector);
        select_shortest_path(imdt_router, dst_router, second_half_pathVector);
        
        
        //join the two vectors
        pathVector.reserve(first_half_pathVector.size() + second_half_pathVector.size());
        pathVector.insert(pathVector.end(), first_half_pathVector.begin(), first_half_pathVector.end());
        pathVector.insert(pathVector.end(), second_half_pathVector.begin() + 1, second_half_pathVector.end());
        
    }
    
    //test printing
    //    cout << "imdt: " << imdt_router << "  path: ";
    //    for(ii = 0; ii < pathVector.size(); ii++){
    //        cout << pathVector[ii] << " ";
    //    }
    //    cout << endl;
    //    
    return imdt_router;
}



int select_vlb_path(const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector){
    /*
        VLB routing with regular and restricted mode support.
        
        For regular, randomly select an intermediate node not within the group.

        For restricted, randomly select an intermediate node from a pool that fulfills a specific condition.
        
        Get min paths from src to imdt, and then from imdt to dst. Depending on
        routing mode, the paths can be df_min or djkstra_min.
        
        Join the paths. Win. 
        
        Returns:    The selected path in pathVector.
                    
                    The selected intermediate node used to generate the path. 
                    This helps to avoid duplicate paths in UGAL routing where multiple VLB paths are considered.
        
        
        The flit is passed for debugging.
        */    
        
    //cout << "inside select_vlb_path_regular for " << "src: " << src_router << " dest: " << dst_router << "  " << endl;
    
    int src_group, dst_group;
    std::pair<int,int> selected_global_link;
    
    std::vector<int> gateway_router_list;
    //int imdt_router;
    int ii;
    bool flag = false;

    std::vector<int> imdt_nodes;
    int no_of_VLB_paths_to_consider = 1;
    imdt_nodes.resize(no_of_VLB_paths_to_consider);   //we are only considering one vlb path 
    
    src_group = src_router / g_a;
    dst_group = dst_router / g_a;
        
    //cout << "src_group: " << src_group << " dst_group: " << dst_group << endl; 
    
    //case 0: src and dst are in the same group.
    //TODO: ship this out to it's own function.
    if (src_group == dst_group){
        //pathVector = {src_router, dst_router};
        
        imdt_nodes[0] = select_vlb_path_inside_group(src_router, dst_router, pathVector);
        
        if (flag){
            cout << "same group. path: ";
            for(ii = 0; ii < pathVector.size(); ii++){
               cout << pathVector[ii] << " ";
            }
        cout << endl;
        }
    }
    
    //case 1: src and dst are in differnet groups 
    else{
        generate_imdt_nodes(f, src_router, dst_router, no_of_VLB_paths_to_consider, imdt_nodes, g_routing_mode, -1);
                        // The last parameter is min_q_len, which is only
                        //relevant for multi-tier routing.
    
        if ((g_routing_mode == "restricted_src_only") || (g_routing_mode == "restricted_src_and_dst") ||(g_routing_mode == "four_hop_restricted")||(g_routing_mode == "four_hop_some_five_hop_restricted")){
            //generate djkstra paths
            generate_vlb_path_from_given_imdt_node(src_router, dst_router, imdt_nodes[ii], pathVector, "djkstra");
        }

        else if (g_routing_mode == "vanilla"){
            generate_vlb_path_from_given_imdt_node(src_router, dst_router, imdt_nodes[ii], pathVector, "generic"); 
        }else{
            cout << "Unsupported routing mode: " << g_routing_mode << " Exiting." << endl;
            exit(-1);
        }
    
    }
    
    //test printing
    if (flag){
       cout << "imdt: " << imdt_nodes[0] << "  path: ";
       for(ii = 0; ii < pathVector.size(); ii++){
           cout << pathVector[ii] << " ";
       }
       cout << endl;
    }

    return imdt_nodes[0];
}

int generate_vlb_path_from_given_imdt_node(int src_router, int dst_router, int imdt_router,  std::vector<int> & pathVector, string shortest_path_mode){
    /*
    This funtion is handy when we are generating the intermediate nodes from an outside
    function following some specific scheme. In that case, this function just accepts that
    intermediate node and generate the complete path. No fuss.
    */
    
    /*
    Skipping error checking. Assumption is that the outer function does all these.    
    */

    if ((shortest_path_mode != "generic") && (shortest_path_mode != "djkstra")){
        cout << "Error! Unsupported shortest_path_mode passed to generate_vlb_path_from_given_imdt_node(). Exiting." << endl;
        exit(-21);  //just a random error value. No further significance.

    }

    std::vector<int> first_half_pathVector;
    std::vector<int> second_half_pathVector;
    
    if (shortest_path_mode == "generic"){    
        select_shortest_path(src_router, imdt_router, first_half_pathVector);
        select_shortest_path(imdt_router, dst_router, second_half_pathVector);
    }
    else if (shortest_path_mode == "djkstra"){
        select_shortest_path_djkstra(src_router, imdt_router, first_half_pathVector);
        select_shortest_path_djkstra(imdt_router, dst_router, second_half_pathVector);        
    }

    //join the two vectors
    pathVector.reserve(first_half_pathVector.size() + second_half_pathVector.size());
    pathVector.insert(pathVector.end(), first_half_pathVector.begin(), first_half_pathVector.end());
    pathVector.insert(pathVector.end(), second_half_pathVector.begin() + 1, second_half_pathVector.end());
    
    return 1; // means nothing at this point.
}



int select_vlb_path_inside_group(int src_router, int dst_router, std::vector<int> & pathVector){
    /*  Both src and dst are within the same group. 
        So generate a vlb path inside the group.
        Basically, randomly pick an intermediate node in the group.
        Then, {src, imdt, dst}
        
        Returns:
            The path itself in pathVector.
            The intermediate node selected to make the path. The info may be useful in UGAL to avoid duplicate paths.
    */
    //safety check
    int src_group = src_router / g_a;
    
    if (src_group != (dst_router / g_a)){
        cout << "Error in calling select_vlb_path_inside_group()" << endl;
        return -1; //error
    }
    
    int imdt_router = src_group * g_a + RandomInt(g_a - 1); //RandomInt gets a number in the range[0,max], inclusive.
    
    while ((imdt_router == src_router) || (imdt_router == dst_router)){
        imdt_router = src_group * g_a + RandomInt(g_a - 1);
    } 
    
    pathVector = {src_router, imdt_router, dst_router};
        
    return imdt_router; //success
}
        
        
int vlb_intermediate_node_vanilla(const Flit *f, int src_router, int dst_router){
    /*
    Just select a node not part of either source group or destination group.
        
    The flit is passed for debugging.
    */
    
    bool flag = false;
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }

    int imdt_node, imdt_group;
    
    int src_group = src_router / g_a;
    int dst_group = dst_router / g_a;
        
    if (flag){
        cout << "inside vlb_intermediate_node_vanilla() " << endl;
        cout << "src: " << src_router << " , src_group: " << src_group << endl;
        cout << "dst: " << dst_router << " , dst_group: " << dst_group << endl;
    }

    imdt_node = RandomInt(g_N - 1); //RandomInt gets a number in the range[0,max], inclusive.
    imdt_group = imdt_node / g_a;

    if(flag){
        cout << "imdt_node: " << imdt_node << " , imdt_group: " << imdt_group << endl;
    }
        
    while ((imdt_group == src_group) || (imdt_group == dst_group)){
        imdt_node = RandomInt(g_N - 1); //RandomInt gets a number in the range[0,max], inclusive. 
        imdt_group = imdt_node / g_a;

        if(flag){
            cout << "imdt_node: " << imdt_node << " , imdt_group: " << imdt_group << endl;
        }
    }
     
    if (flag){
        cout << "returning imdt_node: " << imdt_node << endl;
    }    
    return imdt_node;
}


int vlb_imdt_node_for_five_hop_paths_src_only(const Flit *f, int src_router, int dst_router){
    /*
    Update 28 feb 2019:
    This only looks up src's two-hop neighbors. 
    There is a function further down vlb_imdt_node_for_five_hop_paths_src_and_dst() that
    takes the intersection of src and dst's two-hop neighbors.
    So use that instead.
    */

    /*
    We select an intermediate node which is maximum two hops away from the src_router.    
        [However, the final path to the imdt router will be a DF min path, instead of a
        djkstra shortest path. So chances are that the path will still be three-hops
        anyway.

        Update: that is not a case for Ugal_restricted routing. There a djkstra
        shortest path is taken to the i-node. And we found that it makes a 
        difference for smaller DFs.]
    
    We already have a list of such eligible intermeaidate nodes in g_two_hop_neighbors_vector.
    Just look them up and randomly select one.
    
    The flit is passed for debugging.
    */
        
    bool flag = false;
    
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }
        
    int idx, imdt_node, imdt_group;
    
    int src_group = src_router / g_a;
    int dst_group = dst_router / g_a;
    
    if (flag == true){
        cout << "candidate imdt nodes for src_router " << src_router << " : ";
        for(std::size_t ii = 0; ii < two_hop_neighbors_vector[src_router].size(); ii++){
            cout << two_hop_neighbors_vector[src_router][ii] << " ";
        }
        cout << endl;
    }
        
    idx = RandomInt( two_hop_neighbors_vector[src_router].size() - 1); //RandomInt gets a number in the range[0,max], inclusive.
    imdt_node = two_hop_neighbors_vector[src_router][idx];
    imdt_group = imdt_node / g_a;
    
    if (flag == true){
        cout << "idx: " << idx << "  , imdt_node: " << imdt_node << " , imdt_group: " << imdt_group << endl;
    }
        
    while ((imdt_group == src_group) || (imdt_group == dst_group)){
        if (flag){
            cout << "retrying. ";
        }
        
        idx = RandomInt( two_hop_neighbors_vector[src_router].size() - 1); //RandomInt gets a number in the range[0,max], inclusive.
        imdt_node = two_hop_neighbors_vector[src_router][idx];
        imdt_group = imdt_node / g_a;
        
        if (flag == true){
            cout << "idx: " << idx << "  , imdt_node: " << imdt_node << " , imdt_group: " << imdt_group << endl;
        }
    }
        
    return imdt_node;
}


int vlb_imdt_node_for_five_hop_paths_src_and_dst(const Flit *f, int src_router, int dst_router){

    bool flag = false;
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }

    if (flag){
        cout << "inside vlb_imdt_node_for_five_hop_paths_src_and_dst() for flit: " << f->id << endl;
    }

    std::size_t ii;

    int temp_node;
    int src_group, dst_group;
    
    std::unordered_set<int> imdt_node_set;
    
    src_group = src_router / g_a;
    dst_group = dst_router / g_a;


    std::vector<int> imdt_node_pool;

    
    if(flag){
        cout << "src: " << src_router << endl;
        cout << "src_group: " << src_group << endl;

        cout << "src two hop neighbors: ";
        for(ii = 0; ii < two_hop_neighbors_vector[src_router].size(); ii++ ){
            cout << two_hop_neighbors_vector[src_router][ii] << "    ";
        }
        cout << endl;

        cout << "dst: " << dst_router << endl;
        cout << "dst_group: " << dst_group << endl;
        cout << "dst two hop neighbors: ";
        for(ii = 0; ii < two_hop_neighbors_vector[dst_router].size(); ii++ ){
            cout << two_hop_neighbors_vector[dst_router][ii] << "    ";
        }
        cout << endl;
    }

    for (ii = 0; ii< two_hop_neighbors_vector[src_router].size(); ii++){
        temp_node = two_hop_neighbors_vector[src_router][ii]; 
        if ( (temp_node < (dst_group * g_a)) || (temp_node >= ( (dst_group + 1) * g_a) ) ){
                imdt_node_set.insert(temp_node);
        }
    }


    for (ii = 0; ii< two_hop_neighbors_vector[dst_router].size(); ii++){
        temp_node = two_hop_neighbors_vector[dst_router][ii]; 
        if ( (temp_node < (src_group * g_a)) || (temp_node >= ( (src_group + 1) * g_a) ) ){
                imdt_node_set.insert(temp_node);
        }
    }

    for(auto it = imdt_node_set.begin(); it != imdt_node_set.end(); it++){
        imdt_node_pool.push_back(*it);
    }

    std::sort(imdt_node_pool.begin(), imdt_node_pool.end());

    if(flag){
        cout << "selected imdt nodes pool: ";
        for(ii = 0; ii < imdt_node_pool.size(); ii++ ){
            cout << imdt_node_pool[ii] << "    ";
        }
        cout << endl;
    }
    //return a node randomly from selected intermediate pool 

    int temp = RandomInt(imdt_node_pool.size()-1); 
                //RandomInt(max) selects an int from range(0,max]; max inclusive.
    
    if (flag){
        cout << "rand_int: " << temp << endl;
        cout << "selected node: " << imdt_node_pool[temp] << endl;
    }

    return imdt_node_pool[temp];

}

int vlb_imdt_node_for_four_hop_and_some_five_hop_paths(const Flit *f, int src_router, int dst_router, int five_hop_percentage){
    /*
    Get all four hop paths. 
    Then get a percentage of five hop paths on top of that.

    Python logic was:
    - Get a set of all four-hop imdt nodes.
    - Get a set of all five_hop imdt nodes.
    - Get their set difference. Let's call it extra ones.
    - Shuffle the extra-ones set (vector). Take the specified percentage from it.
    - Return the combined set of four-hop paths and these recently-picked ones.

    Now lets see how this logic can be implemented in Cpp.  

    */


    bool flag = false;
    if (flag){
        cout << "inside vlb_imdt_node_for_four_hop_and_some_five_hop_paths()" << endl;
    }

    //first get the four-hop nodes. We already have a function for that.
    //Take help from there.    
    std::unordered_set<int> four_hop_nodes;
    std::unordered_set<int> five_hop_nodes;
    std::vector<int> unique_five_hop_nodes;
    std::vector<int> eligible_inodes;
 
    int ii;
    int src_group, dst_group;
    int temp_node;
    int cut_off;
    int rdm_idx;
    
    src_group = src_router / g_a;
    dst_group = dst_router / g_a;

    if (flag){
        cout << "src, dst, src_group, dst_group:" << src_router << "," << dst_router << "," << src_group << "," << dst_group << endl;
    }

    //1. Get the global links connected to src. Get their other ends.
    for (ii = g_a-1; ii < g_graph[src_router].size(); ii++){
        if (g_graph[src_router][ii] / g_a != dst_group){
            four_hop_nodes.insert(g_graph[src_router][ii]);
        }
    }

    //2. Get the global links connected to dst. Get their other ends.
    for (ii = g_a-1; ii < g_graph[dst_router].size(); ii++){
        if (g_graph[dst_router][ii] / g_a != src_group){
            four_hop_nodes.insert(g_graph[dst_router][ii]);
        }
    }

    //3. Lookup for common links for src_group, dst_group combo.
    //std::unordered_map < std::pair<int, int>, std::vector<int>, pair_hash > g_group_pair_vs_common_nodes;

    if ( g_group_pair_vs_common_nodes.find( std::make_pair(src_group, dst_group)) != g_group_pair_vs_common_nodes.end()  ){
            std::vector<int> candidates = g_group_pair_vs_common_nodes[std::make_pair(src_group, dst_group)];

            for (ii = 0; ii < candidates.size(); ii++){
                four_hop_nodes.insert(candidates[ii]);
            }
    }

    if (flag){
        cout << "four hop nodes: ";
        print_container(four_hop_nodes); 
        cout << endl;
    }

    //Four hop nodes found. Now gather all the five hop nodes.
    //First, src's two hop neighbors
    for (ii = 0; ii< two_hop_neighbors_vector[src_router].size(); ii++){
        temp_node = two_hop_neighbors_vector[src_router][ii]; 
        if ( (temp_node < (dst_group * g_a)) || (temp_node >= ( (dst_group + 1) * g_a) ) ){
                five_hop_nodes.insert(temp_node);
        }
    }

    //then dest's two-hop neighbors.
    for (ii = 0; ii< two_hop_neighbors_vector[dst_router].size(); ii++){
        temp_node = two_hop_neighbors_vector[dst_router][ii]; 
        if ( (temp_node < (src_group * g_a)) || (temp_node >= ( (src_group + 1) * g_a) ) ){
                five_hop_nodes.insert(temp_node);
        }
    }

    if (flag){
        cout << "five hop nodes: ";
        print_container(five_hop_nodes);   
        cout << endl;
    }

    //Now find the five-hop inodes that are also not four-hop inodes.
    for(auto it = five_hop_nodes.begin(); it != five_hop_nodes.end(); it++){
        if ( four_hop_nodes.find(*it) == four_hop_nodes.end() ){
            unique_five_hop_nodes.push_back(*it);
        }
    }

    if (flag){
        cout << "Unique five hop nodes: ";
        print_container(unique_five_hop_nodes);   
        cout << endl;
    }


    //random-shuffle
    //use the fisher-yates shuffle in traffic.
    fisher_yates_shuffle(unique_five_hop_nodes, unique_five_hop_nodes.size());
    if (flag){
        cout << "Unique five hop nodes after shuffling: ";
        print_container(unique_five_hop_nodes);   
        cout << endl;
    }

    //get the cutoff point
    cut_off = unique_five_hop_nodes.size() * five_hop_percentage / 100 + 1;
    if (flag){
        cout << "cut_off: " << cut_off << endl;
    }


    //generate the eligible inode list
    for (auto it = four_hop_nodes.begin(); it!= four_hop_nodes.end(); it++){
        eligible_inodes.push_back(*it);
    }

    for (ii = 0; ii < cut_off; ii ++){
        eligible_inodes.push_back(unique_five_hop_nodes[ii]);
    }
    if (flag){
        cout << "eligible inodes: ";
        print_container(eligible_inodes);   
        cout << endl;
    }


    //randomly pick one, and return
    rdm_idx = RandomInt(eligible_inodes.size() - 1); //RandomInt includes the limit

    if (flag){
        cout << "random idx:" << rdm_idx << " , selected inode: " << eligible_inodes[rdm_idx] << endl;
    }

    return eligible_inodes[rdm_idx];
}


template <typename T>
void fisher_yates_shuffle(T &container, int size){
  for(int ii = size-1; ii > 0; ii--){
      //pick a random index from 0 to ii
      int jj = RandomInt(ii-1); //RandomInt is inclusive of ii
                                //here, using ii-1 means a node can not be paired with itself.
      //swap 
      std::swap(container[ii], container[jj]);
  }
}


//Updated version. Chooses the four hop vlb paths beased on global links.
int vlb_imdt_node_for_four_hop_paths(const Flit *f, int src_router, int dst_router){
    /*
    Logic:
        1. Get the global links connected to src. Get their other ends.
        2. get the global links connected to dst. Get their other ends.
        3. Lookup to see if there is any node that has global links to both src and dst groups.
            If there exists one, pick that one up.
    
    Also, cache the list of i-nodes for a (src_router, dst_router) combo.
    */    
    bool flag = false;

    std::unordered_set<int> i_nodes;
    std::vector<int> i_node_vector;
    int ii;
    int src_group, dst_group;
    int rdm_idx;

    src_group = src_router / g_a;
    dst_group = dst_router / g_a;

    //1. Get the global links connected to src. Get their other ends.
    for (ii = g_a-1; ii < g_graph[src_router].size(); ii++){
        if (g_graph[src_router][ii] / g_a != dst_group){
            i_nodes.insert(g_graph[src_router][ii]);
        }
    }

    //2. Get the global links connected to dst. Get their other ends.
    for (ii = g_a-1; ii < g_graph[dst_router].size(); ii++){
        if (g_graph[dst_router][ii] / g_a != src_group){
            i_nodes.insert(g_graph[dst_router][ii]);
        }
    }

    //3. Lookup for common links for src_group, dst_group combo.
    //std::unordered_map < std::pair<int, int>, std::vector<int>, pair_hash > g_group_pair_vs_common_nodes;

    if ( g_group_pair_vs_common_nodes.find( std::make_pair(src_group, dst_group)) != g_group_pair_vs_common_nodes.end()  ){
            std::vector<int> candidates = g_group_pair_vs_common_nodes[std::make_pair(src_group, dst_group)];

            for (ii = 0; ii < candidates.size(); ii++){
                i_nodes.insert(candidates[ii]);
            }
    }

    //all potential i_nodes stored in the set. Now randomly pick one.
    i_node_vector.resize(i_nodes.size());

    std::copy(i_nodes.begin(), i_nodes.end(), i_node_vector.begin());

    if (flag){
        cout << "candidate inodes for src " << src_router << " and dst " << dst_router << ":  ";
        print_container(i_node_vector);
    }   

    
    rdm_idx = RandomInt(i_node_vector.size() - 1); //RandomInt includes the limit

    if (flag){
        cout << "random int: " << rdm_idx;
        cout << " node: " << i_node_vector[rdm_idx] << endl;
    }
    return i_node_vector[rdm_idx];
    
}

int vlb_imdt_node_for_three_hop_paths(const Flit *f, int src_router, int dst_router){

    // Logic:
    //     1. Get the 2-hop neighbors of src. See any of them are directly connected to dst. If yes, and not in dst's group,  add as a candidate.
    //     2. Get the 2-hop neighbors of dst. See any of them are directly connected to src. If yes, and not in src's group, add as a candidate.

    bool flag = false;

    std::unordered_set<int> src_neighbors;
    std::unordered_set<int> dst_neighbors;
    std::unordered_set<int> i_node_set;
    std::vector<int> i_node_vector;
    
    //std::vector<int> unique_five_hop_nodes;
 
    int ii;
    int src_group, dst_group;
    int temp_node;
    int rdm_idx;
    int ret_val;

    src_group = src_router / g_a;
    dst_group = dst_router / g_a;


    if(flag){
        cout << "src: " << src_router << endl;
        cout << "src_group: " << src_group << endl;
        cout << "dst: " << dst_router << endl;
        cout << "dst_group: " << dst_group << endl;
    
    }

    //1. Get the global nodes connected to src
    for (ii = g_a-1; ii < g_graph[src_router].size(); ii++){
        src_neighbors.insert(g_graph[src_router][ii]);
    }
    if (flag){
        cout << "src neighbors:";
        print_container(src_neighbors);
    }

    //2. Get the global nodes connected to dst
    for (ii = g_a-1; ii < g_graph[dst_router].size(); ii++){
        dst_neighbors.insert(g_graph[dst_router][ii]);
    }
    if (flag){
        cout << "src neighbors:";
        print_container(dst_neighbors);
    }
    


    //3. Get the 2hop neighbors of src. If they are not in dst's group,
    //check to see if they are directly connected to dst. If yes, add
    //as a candidate.
    for(ii = 0; ii < two_hop_neighbors_vector[src_router].size(); ii++){
        temp_node =  two_hop_neighbors_vector[src_router][ii];
        if ( (temp_node / g_a) != dst_group ){
            if (dst_neighbors.find(temp_node) != dst_neighbors.end()){
                i_node_set.insert(temp_node);
            }
        }
    }

    //4. Get the 2hop neighbors of dst. If they are not in src's group,
    //check to see if they are directly connected to src. If yes, add
    //as a candidate.
    for(ii = 0; ii < two_hop_neighbors_vector[dst_router].size(); ii++){
        temp_node =  two_hop_neighbors_vector[dst_router][ii];
        if ( (temp_node / g_a) != src_group ){
            if (src_neighbors.find(temp_node) != src_neighbors.end()){
                i_node_set.insert(temp_node);
            }
        }
    }

    if (flag){
        cout << "candidate inode set for src " << src_router << " and dst " << dst_router << ":  ";
        print_container(i_node_set);
    }

    //all potential i_nodes stored in the set. Now randomly pick one.
    i_node_vector.resize(i_node_set.size());

    std::copy(i_node_set.begin(), i_node_set.end(), i_node_vector.begin());

    if (flag){
        cout << "candidate inodes for src " << src_router << " and dst " << dst_router << ":  ";
        print_container(i_node_vector);
    }   

    
    if (i_node_vector.size() != 0){
        rdm_idx = RandomInt(i_node_vector.size() - 1); //RandomInt includes the limit
        ret_val =  i_node_vector[rdm_idx]; 
    }else{
        ret_val =vlb_imdt_node_for_four_hop_paths(f, src_router, dst_router);
    }

    
    return ret_val;
    
  

    
}

void UGAL_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject){
    /*
    Vanilla UGAL_l routing function.
        
    ******************************
    source routing. 
    If it is the source router: 
                 Make the routing decision (MIN or VLB).     
                 Save the complete path in the flit. 
                 Forward the filt to the next hop.
    If it is not the source router, then:
                 Just look up the saved path in the flit
                 And forward to the next hop.
    That's it!
    ******************************
    
    Things to decide: 
        No of min paths and no of vlb paths to consider while path selection.
        Check default dragonfly implementation in booksim + dragonfly literature to get the numbers.
            Update: default Booksim implementaiton compared one min and one non-min paths.
    
    As usual,
    The routing function needs to find:
        - an output port
        - an output vc
        
    For the output port:
        - First check if it is an injection port. If yes, do accordingly.
        
        - Then check if it is the destination router. If yes, then find the port
         to the destination PE.
         
        - Then check if it is the source router. If yes:
            - if src and dst are in the same group, route them normally.
                    (This can be changed too. Later think about it.)
            
            - else, make the routing decision. generate the full path accordingly.
            
            - Save the whole path in the flit 
            
        - Based on the hop count, find the port id to the next router.
        - Increase hop-count.
        
    For the output vc:
        - give a new vc on each hop.
            - For non-minimal routing, max hop-count is 6. So we need 6 + 1(for injection) vcs.
        - For flit injection, assign to vc 0. 
            - Will that lead to injection vs congestion? Check.
            - In that case, we can just randomly assign a vc. We did it before, Probably not harmful.
    
    So two functions needed:
        - A function to generate the complete path for the flit, given a pair of src and dst.
        - A function for finding the port to the next hop given the flit's hop count, 
        saved path, current and next routers.
    */
    
    //First check if it is an injection port. If yes, do accordingly.
    
    bool flag = false;
    
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }
    
    if (flag){
        cout << "inside UGAL_dragonflyfull() for flit: " << f-> id << endl;
        cout << "routing mode: " << g_routing_mode << endl;
        cout << "hop_count: " << f->hop_count;
        cout << " vc: " << f->vc << endl;
        
    }
    
    
    if(inject) {
        g_total_flit += 1;

        outputs->Clear( ); //doesn't matter really. If the flit is at injection,
                            //means it was just generated, so the outputset is empty anyway.

        int inject_vc= RandomInt(gNumVCs-1);
        outputs->AddRange(-1, inject_vc, inject_vc);
        
        if (flag){
            cout << "injecting flit: " << f->id << " with vc: " << inject_vc << endl;
        }
          
        return;
    }
    
    
    //gather necessary info from the flit
    outputs -> Clear();
    
    int current_router = r->GetID();
    if (flag){
        cout << "current router:" << current_router << endl;
    }

    int prev_router = -1;
    int next_router = -1;
    
    int out_port = -33;    //instead of setting all the default error values to -1, we used different values in different places. Works kinda like different error codes, helps in debugging.
    
    int out_vc = 0;
    
    int dst_PE = f->dest;
    int dst_router = dst_PE / g_p;
    
    //Then check if it is the destination router. 
    //If yes, then find the port to the destination PE.
    if(current_router == dst_router){
        out_vc = RandomInt(gNumVCs-1);
            //This could be a static VC as well. But a PE is basically a exit drain for packets,
            //so differentiating between VCs doesn't seem neceesary. Rather just draining them as fast as possible.
        
        //find the port that goes to the destination processing node
        out_port = dst_PE % g_p;
        
        if (flag){
            cout << "destination router. ejecting through port " << out_port << " through vc " << out_vc << endl;
        }
    }
    
    else if (f->hop_count == 0){   //source router
        //if it is at the source router, then the input port must be a PE. Check if it is any different.
        if (flag){
            cout << "inside source router ..." << endl;
        }
        
        if (in_channel >= g_p){
            cout << "EXCEPTION! Source router but port is not smaller than p" << endl;
            cout << "flitID: " << f->id << " src: " << f->src << " dest: " << f->dest  << " rID: " << r->GetID() << endl;
    
            cout << "p: " << g_p << " in_channel: " << in_channel << endl;
        }
        
    
        //actual UGAL routing needs to happen here.
        
        std::vector<int> pathVector;
        
        if (flag){
            cout << "dest router: " << dst_router << endl;
        }
        
        int temp; 
        
        temp = select_UGAL_path(r, f, current_router, dst_router, pathVector, g_routing_mode);
        
        if (flag){
            cout << "select_ugal_path() returned: " << temp << endl;
            cout << "the path returned is: ";
                for(std::size_t ii = 0; ii < pathVector.size(); ii++){
                    cout << pathVector[ii] << " ";
                }
                cout << endl;
                
        }
        //Save the whole path in the flit
        f->path = std::move(pathVector);
        
        //now get the port to the next hop node 
        out_port = find_port_to_node(current_router, f->path[1]);
        
        if(flag){
            cout << "port to next router is: " << out_port << endl;
        }
        
        //assign vc. first hop, so vc 0.
        out_vc = 0;
        
        //increase hop_count
        f->hop_count += 1;
    
    }else{  //neither src nor dst router. Packet in flight.
        //get current hop count 
        
        //get port to the next hop node
        out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
        
        //assign vc 
        if (g_vc_allocation_mode == "incremental"){
            out_vc = f->hop_count;
        }
        else{
            prev_router = f->path[f->hop_count - 1];
            next_router =  f->path[f->hop_count + 1];
            out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
        }

        if (flag){
            cout << "port to next hop: " << out_port << " through vc: " << out_vc << endl; 
        }
        
        //increase hop_count
        f->hop_count += 1;
        
    }

    //finally, build the output set
    outputs->AddRange( out_port, out_vc, out_vc );
    
    if (flag){
        cout << "leaving UGAL_dragonflyfull() for flit: " << f-> id << endl;
    }
    
}


int allocate_vc(const Flit *f, int prev_router, int current_router, int next_router, int current_vc){
    
    // Previously we just used incremental VC allocation. That got us burned in
    // Valhalla. So finally writing a function to optimize VC allocation that avoids
    // any potential deadlocks. 
       
    // Logic:
    // If packet in src_router: vc -> 0

    // Else: //packet not in src_router
    //     If the packet just arrived in this group: vc += 1

    //     Else:   //not src, not just arrived in the group
    //         If the next hop is local: vc += 1

    //         Else: //not src, not just arrived in this group, next hop global
    //             Do nothing.

    // //The src_router or dst_router cases will be handled outside anyway.
    // //So this funciton will only be called when the packet is en route.   
    
    assert((current_vc != -1) && (prev_router != -1) && (current_router != -1) && (next_router != -1));

    bool flag = false;
    // if ((f->id > 82959) && (f->id < 82969)){
    //     flag = true;
    // }

    if (flag){
        cout << "allocate_vc called for ";
        cout << "  flit: " << f->id;
        cout << "  prev_router: " << prev_router;
        cout << "  current_router: " << current_router;
        cout << "  next_router: " << next_router;
        cout << "  current_vc: " << current_vc;
        cout << endl;
    }

    int return_vc = -1;

    //Did the packet just arrive in the group?
    if ((prev_router / g_a) != (current_router / g_a)){ //new group arrival
        return_vc = current_vc + 1;
        if (flag){
            cout << "New arrival. Vc set to: " << return_vc << endl;
        }
    }
    else{   //not new arrival
        if ((current_router / g_a) == (next_router / g_a)){ //local link
            return_vc = current_vc + 1;
            if (flag){
                cout << "Local link. Vc set to: " << return_vc << endl;       
            }
        }
        else{   //global link
            return_vc = current_vc;
            if (flag){
                cout << "Global link. Vc set to: " << return_vc << endl;
            }
        }
    }

    return return_vc;
}


void PAR_dragonflyfull( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject){
    /*
    PAR: Prograssive Adaptive routing function.
        
    ******************************
    source routing. 
    If it is the source router: 
                 Make the routing decision (MIN or VLB).     
                 Save the complete path in the flit.
                 Mark if the min path was taken. 
                 Forward the filt to the next hop.
    If it is on second router on its path, then:
                 Check if it is already out of its source group. If yes, do nothing.
                 Check if the non-min path was taken in the source router. If yes, do nothing.
                 Else,
                    (Min router was chosen in the non-gateway source router.)
                    Re-evaluate the routing decision. 
                    If necessary, move to non-Min path. In that case:
                        Save the new path in the flit.
                        Bump the VC by 1.
                 Forward to the next hop.
    Otherwise:
                 Just look up the saved path in the flit
                 And forward to the next hop.
    That's it!
    ******************************
    
    Things to decide: 
        No of min paths and no of vlb paths to consider while path selection.
        Check default dragonfly implementation in booksim + dragonfly literature to get the numbers.
            Update: default Booksim implementaiton compared one min and one non-min path.
    
    As usual,
    The routing function needs to find:
        - an output port
        - an output vc
        
    For the output port:
        - First check if it is an injection port. If yes, do accordingly.
        
        - Then check if it is the destination router. If yes, then find the port
         to the destination PE.
         
        - Then check if it is the source router. If yes:
            - make the routing decision. generate the full path accordingly.
            - Save the whole path in the flit 
            
        - Then check if it is the second router on the path. Re-evaluate routing decision if needed.

        - Based on the hop count, find the port id to the next router.
        - Increase hop-count.
        
    For the output vc:
        - give a new vc on each hop.
            - For non-minimal routing, max hop-count is 6. So we need 6 + 1(for injection) vcs.
        - For flit injection, assign to vc 0. 
            - Will that lead to injection vs congestion? Check.
            - In that case, we can just randomly assign a vc. We did it before, Probably not harmful.
    
    So two functions needed:
        - A function to generate the complete path for the flit, given a pair of src and dst.
        - A function for finding the port to the next hop given the flit's hop count, 
        saved path, current and next routers.
    */
    
    //First check if it is an injection port. If yes, do accordingly.
    
    bool flag = false;
    
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }
    
    if (flag){
        cout << "inside PAR_dragonflyfull() for flit: " << f-> id << endl;
        cout << "routing mode: " << g_routing_mode << endl;
    }
    
    
    if(inject) {
        outputs->Clear( ); //doesn't matter really. If the flit is at injection,
                            //means it was just generated, so the outputset is empty anyway.

        int inject_vc= RandomInt(gNumVCs-1);
        outputs->AddRange(-1, inject_vc, inject_vc);
        
        if (flag){
            cout << "injecting flit: " << f->id << " with vc: " << inject_vc << endl;
        }
          
        return;
    }
    
    
    //gather necessary info from the flit
    outputs -> Clear();
    
    int current_router = r->GetID();
    if (flag){
        cout << "current router:" << current_router << endl;
    }
    
    int out_port = -33;    //instead of setting all the default error values to -1, we used different values in different places. Works kinda like different error codes, helps in debugging.
    
    int out_vc = 0;
    int next_router = -1;
    int prev_router = -1;
    
    int dst_PE = f->dest;
    int dst_router = dst_PE / g_p;
    
    //Then check if it is the destination router. 
    //If yes, then find the port to the destination PE.
    if(current_router == dst_router){
        out_vc = RandomInt(gNumVCs-1);
            //This could be a static VC as well. But a PE is basically a exit drain for packets,
            //so differentiating between VCs doesn't seem neceesary. Rather just draining them as fast as possible.
        
        //find the port that goes to the destination processing node
        out_port = dst_PE % g_p;
        
        if (flag){
            cout << "destination router. ejecting through port " << out_port << " through vc " << out_vc << endl;
        }
    }
    
    else if (f->hop_count == 0){   //source router
        //if it is at the source router, then the input port must be a PE. Check if it is any different.
        if (flag){
            cout << "inside source router ..." << endl;
        }
        
        if (in_channel >= g_p){
            cout << "EXCEPTION! Source router but port is not smaller than p" << endl;
            cout << "flitID: " << f->id << " src: " << f->src << " dest: " << f->dest  << " rID: " << r->GetID() << endl;
    
            cout << "p: " << g_p << " in_channel: " << in_channel << endl;
        }
        
    
        //actual UGAL routing needs to happen here.
        
        std::vector<int> pathVector;
        
        if (flag){
            cout << "dest router: " << dst_router << endl;
        }
        
        int temp; 
        
        temp = select_UGAL_path(r, f, current_router, dst_router, pathVector, g_routing_mode);
        
        if (flag){
            cout << "select_ugal_path() returned: " << temp << endl;
            cout << "the path returned is: ";
                for(std::size_t ii = 0; ii < pathVector.size(); ii++){
                    cout << pathVector[ii] << " ";
                }
                cout << endl;
                
        }
        //Save the whole path in the flit
        f->path = std::move(pathVector);
        
        //now get the port to the next hop node 
        out_port = find_port_to_node(current_router, f->path[1]);
        
        if(flag){
            cout << "port to next router is: " << out_port << endl;
        }
        
        //assign vc. first hop, so vc 0.
        out_vc = 0;
        
        //increase hop_count
        f->hop_count += 1;
    
    }
  
    else if ((f->hop_count == 1) && (f->PAR_need_to_revaluate == true)) {
        //if it is out of source group already, PAR is not needed
        int src_router = f->src / g_p;
        int src_group = src_router / g_a;

        if ( (current_router >= (src_group*g_a)) && (current_router < (src_group*g_a + g_a) ) ){
            //Still in the source group. PAR code goes here.

            std::vector<int> pathVector;
            int temp; 
        
            temp = select_UGAL_path(r, f, current_router, dst_router, pathVector, g_routing_mode);
            
            if (flag){
                //cout << "inside PAR_dragonflyfull() for flit: " << f-> id << endl;
                cout << "the complete path stored in the flit is: ";
                    for(std::size_t ii = 0; ii < f->path.size(); ii++){
                        cout << f->path[ii] << " ";
                    }
                    cout << endl;
            }

            //Path returned, now replace the old path with the new one.
            
            //Because we are saving the complete path in the flit, 
            //it makes  sense to include the previous node as well.
            pathVector.insert(pathVector.begin(), src_router);

            f->path = std::move(pathVector);

            out_port = find_port_to_node(current_router, f->path[f->hop_count + 1]);
        
            //assign vc 
            if (g_vc_allocation_mode == "incremental"){
                out_vc = f->hop_count;
            }
            else{
                prev_router = f->path[f->hop_count - 1];
                next_router =  f->path[f->hop_count + 1];
                out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
            }

            f->hop_count += 1;

            if (flag){
                cout << "values returned from PAR: " << temp << endl;
                cout << "the complete path: ";
                    for(std::size_t ii = 0; ii < f->path.size(); ii++){
                        cout << f->path[ii] << " ";
                    }
                    cout << endl;
                cout << "out port: " << out_port << endl;
                cout << "vc: " << out_vc << endl;
                cout << "hop count: " << f->hop_count << endl;
            }
        
            

        }else{
            //PAR path correction not necessary. Business as usual.
            //get port to the next hop node
            out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
            
            //assign vc 
            if (g_vc_allocation_mode == "incremental"){
                out_vc = f->hop_count;
            }
            else{
                prev_router = f->path[f->hop_count - 1];
                next_router =  f->path[f->hop_count + 1];
                out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
            }

            
            if (flag){
                cout << "port to next hop: " << out_port << " through vc: " << out_vc << endl; 
            }
            
            //increase hop_count
            f->hop_count += 1;
            
        }

        
    }
    else{  //neither src nor dst router. Packet in flight.
        //get current hop count 
        
        //get port to the next hop node
        out_port = find_port_to_node(current_router, f->path[ f->hop_count + 1]);
        
        //assign vc 
        //assign vc 
        if (g_vc_allocation_mode == "incremental"){
            out_vc = f->hop_count;
        }
        else{
            prev_router = f->path[f->hop_count - 1];
            next_router =  f->path[f->hop_count + 1];
            out_vc = allocate_vc(f, prev_router, current_router, next_router, f->vc);
        }

        if (flag){
            cout << "port to next hop: " << out_port << " through vc: " << out_vc << endl; 
        }
        
        //increase hop_count
        f->hop_count += 1;
    }

    //finally, build the output set
    outputs->AddRange( out_port, out_vc, out_vc );
    
    if (flag){
        cout << "leaving PAR_dragonflyfull() for flit: " << f-> id << endl;
    }    
}


int select_UGAL_path( const Router *r, const Flit *f, int src_router, int dst_router, std::vector<int> & pathVector, string routing_mode){
    /*
        Vanilla UGAL_L routing, but a twist can be added using routing_mode.
        
        routing_mode is passed by the user, 
        this dictates how the random intermediate node(s) will be chosen.
        Current supported modes are:
                    //UGAL_L
                    //UGAL_L_two_hop
                    //UGAL_L_threshold
                        Update: More modes supported. Check inside the setRoutingMode() function.
        
        //flit is only passed for debugging
        
        //generate one min path
        //generate one VLB path
        //multiply the Q-length for each path with the paths hop count
        //choose the path with the smallest value
    */
        
    bool flag = false;
    
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }
        
    if (flag){
        cout << "inside select_UGAL_path() for flit " << f->id << endl;
    }
    
    int no_of_MIN_paths_to_consider = 1;        
                            //At present it doesn't support any other value.
                            //But can change this in future

    int no_of_VLB_paths_to_consider = 1;  //can change this in future 
    
    int ii, temp, chosen_pathID;
        
    std::vector< std::vector<int> > paths;
    std::vector<int> imdt_nodes;
    
    int min_q_len;
    int chosen_tier = 0; //only useful for multi-tiered routing
    
    //this block is to take care fo the PAR second-hop revaluation special case.
    if (f->PAR_need_to_revaluate == true){
        no_of_MIN_paths_to_consider = 1;
            //at this point, it is redundant. 
            //But in future if we support more that one min paths,
            //the resetting it to 1 here would be crucial.
    }


    paths.resize(no_of_MIN_paths_to_consider + no_of_VLB_paths_to_consider); 
                        //1 min path, the rest non-min paths
    imdt_nodes.resize(no_of_VLB_paths_to_consider, -1);
        
    //individual paths need not be initialized, as select_shortest_path() and select_vlb_path_regular() initialize the vectors themselves.
    
        
    //TODO: this might be more generalized to select more than one min paths
    
    //get the shortest path
    if (f->PAR_need_to_revaluate == true){
        if (flag){
            cout << "using djkstra shortest path. " << endl;
        }
        temp = select_shortest_path_djkstra(src_router, dst_router, paths[0]);
    }
    else{
        temp = select_shortest_path(src_router, dst_router, paths[0]);
    }

    if (flag){
        cout << "select_shortest_path() returned " << temp << endl;
    }

    if (temp != 1){
        cout << "Error! No min path found. Exiting." << endl;
        exit(-1);
    }
    
    //get the q_len to the shortest path
    min_q_len = find_port_queue_len_to_node(r, paths[0][0], paths[0][1]);
    

    //Now, get the non-min paths
    
    int src_group, dst_group;

    src_group = src_router / g_a;
    dst_group = dst_router / g_a;

    //First, check if src and dst are in the same group
    if (src_group == dst_group){
    //if(0 == 1){
        if (no_of_VLB_paths_to_consider > (g_a - 2)){
            cout << "Error: not enough in-group nodes to choose from. Exiting." << endl;
            exit(-9); //-9 is just an error flag. no other meaning.
        }

        //select in-group nodes only as imdt nodes
        generate_in_group_vlb_paths(f, src_router, dst_router, no_of_VLB_paths_to_consider, paths.begin()+no_of_MIN_paths_to_consider, paths.end());

    }else{

        chosen_tier = generate_imdt_nodes(f, src_router, dst_router, no_of_VLB_paths_to_consider, imdt_nodes, routing_mode, min_q_len);
        if (flag){
            cout << "chosen_tier " << chosen_tier << endl;
            cout << "selected imdt nodes:";
            for(std::size_t jj = 0; jj < imdt_nodes.size(); jj++){
                cout << imdt_nodes[jj] << " ";
            }
            cout << endl;
        }

        for(ii = 0; ii < no_of_VLB_paths_to_consider; ii++){


            if ((g_routing_mode == "restricted_src_only") || (g_routing_mode == "restricted_src_and_dst") || ( routing_mode == "four_hop_restricted") || ( routing_mode == "three_hop_restricted")  || ( routing_mode == "four_hop_some_five_hop_restricted")){
           
                //generate djkstra paths
                generate_vlb_path_from_given_imdt_node(src_router, dst_router, imdt_nodes[ii], paths[ii+no_of_MIN_paths_to_consider], "djkstra");
            }
            else{
                generate_vlb_path_from_given_imdt_node(src_router, dst_router, imdt_nodes[ii], paths[ii+no_of_MIN_paths_to_consider], "generic"); 
                            //the first no_of_MIN_paths_to_consider element in paths[] array is used for min path
            }
        }
    }

    if (flag){
        cout << "generate_vlb_path_from_given_imdt_node() returned " << endl;
        cout << "genrated paths:" << endl;
        for(std::size_t jj = 0; jj < paths.size(); jj++){
            cout << "path " << jj << " : ";
            for(std::size_t kk=0; kk < paths[jj].size(); kk++){
                cout << paths[jj][kk] << " ";
            }
            cout << endl;
        }
    }
    
    //paths generated, now compare Q length and select one
    if (g_ugal_local_vs_global_switch == "local"){
        chosen_pathID = make_UGAL_L_path_choice(r, f, no_of_MIN_paths_to_consider, no_of_VLB_paths_to_consider, paths, chosen_tier);
    }else if (g_ugal_local_vs_global_switch == "global"){
        chosen_pathID = make_UGAL_G_path_choice(r, f, no_of_MIN_paths_to_consider, no_of_VLB_paths_to_consider, paths, chosen_tier);
    }else{
        cout << "Error. invalid g_ugal_local_vs_global_switch value: " << g_ugal_local_vs_global_switch << endl;
        exit(-1);
    }
    
    if (flag){
        cout << "chosen ugal_l path id: " << chosen_pathID << endl;
        cout << "chosen path: ";
        for(std::size_t kk=0; kk < paths[chosen_pathID].size(); kk++){
            cout << paths[chosen_pathID][kk] << " ";
        }
        cout << endl;
    }

    //decide here if a min path was selected in UGAL. If yes, mark it for PAR routing.
        
    if (chosen_pathID < no_of_MIN_paths_to_consider){
        f->PAR_need_to_revaluate = true;
                //for PAR routing only. For others, it will have no effect.
    }


    //Now copy the chosen path in pathVector
    pathVector = paths[chosen_pathID];  //vector assignment operator invokes a deep copy, so safe.
    
    return 1;   //to indicate success, no other significance at this moment.
}

int generate_in_group_vlb_paths(const Flit *f, int src_router, int dst_router, int no_of_nodes_to_generate, std::vector< std::vector<int> >::iterator start, std::vector< std::vector<int> >::iterator finish){

    //we already checked that there are enough in group nodes to choose from.

    //Repeat no_of_nodes_to_generate times
        //Randomly choose an inter-group nodes.
        //if it is equal to src, dst, or any other node previously encountered, discard it and re-do.
    bool flag = false;

    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }

    if(flag){
        cout << "inside generate_in_group_vlb_paths() for flit " << f->id << endl;
        cout << "src_router: " << src_router << " , dst_router: " << dst_router << endl;   
    }

    std::unordered_set <int> node_cache;
    int ii;
    int src_group = src_router / g_a;
    int imdt_node = -1;

    for(ii = 0; ii < no_of_nodes_to_generate; ii++){
        while( (imdt_node == -1) ||(imdt_node == src_router) || (imdt_node == dst_router) ||( node_cache.find(imdt_node) != node_cache.end())){
                imdt_node = src_group * g_a + RandomInt(g_a - 1); //RandomInt gets a number in the range[0,max], inclusive.
                
                
                if (flag){
                    cout << "imdt_node generated: " << imdt_node << endl;
                }
        }
        node_cache.insert(imdt_node);
    }
    
    if (flag){
        cout << "generated imdt nodes: ";
        for(auto it = node_cache.begin(); it != node_cache.end(); it++){
            cout << *it << " ";
        }
        cout << endl;
    }

    auto it2 = node_cache.begin();
    for(auto it = start; it != finish; it++, it2++){
        (*it).push_back(src_router);
        (*it).push_back(*it2);
        (*it).push_back(dst_router);
    }

    if(flag){
        cout << "returning from generate_in_group_vlb_paths()" << endl;
    }


    return 0; //no meaning.
}


int generate_imdt_nodes(const Flit *f, int src_router, int dst_router, int no_of_nodes_to_generate, std::vector<int> & imdt_nodes, string routing_mode, int min_q_len){
    /*
    A function that generates a list of intermeaidate nodes to generate VLB paths through, 
    for a given src_router and dst_router.
    
    Depending on routing_mode, different functions are called to generate intermeaidate nodes.
    
    The general constraints are:
        1) The generated nodes need to be unique
        2) A intermediate group can not belong to the same group as src or dst.
    
    The flit is passed just for debugging.
    */
    
    bool flag = false;

    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }

    std::unordered_set<int> node_cache;
    int ii;
    int imdt_node = -1;

    int chosen_tier = 0; //only needed for multi-tiered routing.

    if (flag){
        cout << "no_of_nodes_to_generate: " << no_of_nodes_to_generate << endl;
        cout << "routing mode: " << routing_mode << endl;
    }    

    //routing modes: vanilla_ugal, two_hop, threshold,  not_applicable
    for(ii = 0; ii < no_of_nodes_to_generate; ii++){
        while( (imdt_node == -1) || ( node_cache.find(imdt_node) != node_cache.end() ) ){

            if (routing_mode == "vanilla" ){
                imdt_node = vlb_intermediate_node_vanilla(f, src_router, dst_router);
                if (flag){
                    cout << "intermediate node generator function returned: " << imdt_node << endl; 
                }
            }


            else if( (routing_mode == "restricted_src_and_dst") ) {
                imdt_node = vlb_imdt_node_for_five_hop_paths_src_and_dst(f, src_router, dst_router);
                if (flag){
                    cout << "intermediate node generator function returned: " << imdt_node << endl; 
                }
            }

            else if( (routing_mode == "two_hop") || (routing_mode == "restricted_src_only") ){
                //imdt_node = vlb_imdt_node_for_five_hop_paths_src_only(f, src_router, dst_router);
                imdt_node = vlb_imdt_node_for_five_hop_paths_src_only(f, src_router, dst_router);
                if (flag){
                    cout << "intermediate node generator function returned: " << imdt_node << endl; 
                }
            }

            else if (routing_mode == "four_hop_restricted") {
                imdt_node = vlb_imdt_node_for_four_hop_paths(f, src_router, dst_router);
                if (flag){
                    cout << "intermediate node generator function returned:" << imdt_node << endl; 
                }
            }

            else if (routing_mode == "three_hop_restricted") {
                imdt_node = vlb_imdt_node_for_three_hop_paths(f, src_router, dst_router);
            }
            else if (routing_mode == "four_hop_some_five_hop_restricted") {
                imdt_node = vlb_imdt_node_for_four_hop_and_some_five_hop_paths(f, src_router, dst_router, g_five_hop_percentage);
                if (flag){
                    cout << "intermediate node generator function returned:" << imdt_node << endl; 
                }
            }

            else if(routing_mode == "threshold"){
                //compare the q_len of min path with the threshold value.
                //if it is smaller, keep using two-hop nodes.
                //if it is greater, revert to vanilla UGAL.
                if (flag){
                    cout << "min_q_len " << min_q_len << endl; 
                    cout << "g_threshold " << g_threshold << endl; 
                
                }
                if (min_q_len < g_threshold){
                    //imdt_node = vlb_imdt_node_for_five_hop_paths_src_only(f, src_router, dst_router);
                    imdt_node = vlb_imdt_node_for_five_hop_paths_src_only(f, src_router, dst_router);
                    if (flag){
                        cout << "intermediate node generator function returned: " << imdt_node << endl;        
                    }
                }else{
                    imdt_node = vlb_intermediate_node_vanilla(f, src_router, dst_router);
                    if (flag){
                        cout << "intermediate node generator function returned:" << imdt_node << endl; 
                    }
                }
                
            }
            
            
            else{
                cout << "Error! Unsupported routing_mode: " << routing_mode << " ! Exiting." << endl;
                exit(-1);
            }
        }
        node_cache.insert(imdt_node);
        imdt_nodes[ii] = imdt_node;
    }
    return chosen_tier; 
                        //helpful for stats collection in multi-tiered routing. 
                        //For other routings, the default value of 0 will be passed back.

}




int vlb_imdt_node_for_four_hop_paths_old(const Flit *f, int src_router, int dst_router){

    //cout << "inside vlb_imdt_node_for_four_hop_paths()" << endl;
    bool flag = false;
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }


    std::size_t ii;

    std::vector<int> imdt_node_pool;

    if (flag){
        cout << "inside vlb_imdt_node_for_four_hop_paths() for flit: " << f->id << endl;
    }

    //select src's eligible neighbor list
    //select dst's eligible neighbor list
    //get their intersection
    // cout << "tier 4." << endl;
    // cout << "src: " << src_router << endl;
    
    // cout << "src one hop neighbors: ";
    // for(ii = 0; ii < one_hop_neighbors_vector[src_router].size(); ii++ ){
    //     cout << one_hop_neighbors_vector[src_router][ii] << "    ";
    // }
    // cout << endl;
    
    if (flag){
        cout << "src two hop neighbors: ";
        for(ii = 0; ii < two_hop_neighbors_vector[src_router].size(); ii++ ){
            cout << two_hop_neighbors_vector[src_router][ii] << "    ";
        }
        cout << endl;
    }
    // cout << "dst: " << dst_router << endl;
  
    // cout << "dst one hop neighbors: ";
    // for(ii = 0; ii < one_hop_neighbors_vector[dst_router].size(); ii++ ){
    //     cout << one_hop_neighbors_vector[dst_router][ii] << "    ";
    // }
    // cout << endl;
    
    if (flag){
        cout << "dst two hop neighbors: ";
        for(ii = 0; ii < two_hop_neighbors_vector[dst_router].size(); ii++ ){
            cout << two_hop_neighbors_vector[dst_router][ii] << "    ";
        }
        cout << endl;
    }
    //The way things are now, two_hop_neighbors_vector already includes all nodes
    //in one_hop_neighbors_vector. And as we are not differentiating between 
    //non-min paths of lenght 2,3 and 4. So we can just take intersections of 
    //the two-hop-neighbors and get it done with. 
    if (flag){
        cout << "two_hop_neighbors_vector[src_router].size(): " << two_hop_neighbors_vector[src_router].size() << endl;
        cout << "two_hop_neighbors_vector[dst_router].size(): " << two_hop_neighbors_vector[dst_router].size() << endl; 
    }

    imdt_node_pool.resize( two_hop_neighbors_vector[src_router].size() > two_hop_neighbors_vector[dst_router].size() ?  two_hop_neighbors_vector[src_router].size() : two_hop_neighbors_vector[dst_router].size() );

    if(flag){
        cout << "imdt_node_pool resized to: " << imdt_node_pool.size() << endl;
    }

    if(flag){
        cout << "crash now" << endl;
    }

    auto it = std::set_intersection(two_hop_neighbors_vector[src_router].begin(),
        two_hop_neighbors_vector[src_router].end(), two_hop_neighbors_vector[dst_router].begin(),
        two_hop_neighbors_vector[dst_router].end(),
        imdt_node_pool.begin()
        );

    imdt_node_pool.resize(it - imdt_node_pool.begin());

    if (flag){
        cout << "selected imdt nodes pool: ";
        for(ii = 0; ii < imdt_node_pool.size(); ii++ ){
            cout << imdt_node_pool[ii] << "    ";
        }
        cout << endl;
    }

    //two-hop-neighbors intersection will already discard the nodes
    //that are part of src and dst's group. So no need to check for those.

    //return a node randomly from selected intermediate pool 

    int temp = RandomInt(imdt_node_pool.size()-1); 
                //RandomInt(max) selects an int from range(0,max]; max inclusive.
    if (flag){
        cout << "rand_int: " << temp << endl;
        cout << "selected node: " << imdt_node_pool[temp] << endl;
    }

    return imdt_node_pool[temp];
}



int make_UGAL_L_path_choice(const Router *r, const Flit *f, int no_of_min_paths_to_consider, int no_of_VLB_paths_to_consider, std::vector< std::vector<int> > paths, int chosen_tier){
    /* 
    At this moment, we are just considering min path weight as 1
    and non-min path weight as 2.
        Update: After update, now it is being controlled by the global variable
        g_ugal_multiply_mode. Based on its value, the multipliers can be 1 vs 2
        or min_hop_count vs vlb_hop_count.
    
    The flit is passed for debugging. 

    chosen_tier is useful for stat generation in multi-tiered routing only.
    */
    
    bool flag = false;
    if (f->id == FLIT_TO_TRACK){
        flag = true;
    }
    
    int min_shortest_path_weight = 9999; //arbitrary large number
    int min_VLB_path_weight = 9999;
    
    int ii, jj;
    int q_len;
    int temp_q_len;
    
    int selected_min_path_id = -1;  //dummy values. If they are not get changed then it means we have a problem.
    int selected_VLB_path_id = -1;    
    int chosen_path_id;

    int selected_min_path_hop_count = 0;
    int selected_VLB_path_hop_count = 0;

    int chosen; // 0 for min, 1 for non-min     

    int min_multiplier, vlb_multiplier;


    //src_group and dst_group are only needed here for stat collection.
    int src_group = ((f->src) / g_p ) / g_a; 
    int dst_group = ((f->dest) / g_p ) / g_a; 
    
    //go through the min paths and select the one with the lowest Q length
    if (flag){
        cout << "min paths q len:" << endl;
    }
    for(ii = 0; ii< no_of_min_paths_to_consider; ii++){
        q_len = find_port_queue_len_to_node(r, paths[ii][0], paths[ii][1]);
        if (flag){
            cout << "path " << ii << " q_len: " << q_len << endl;
        }
        if (q_len < min_shortest_path_weight){
            selected_min_path_id = ii;
            min_shortest_path_weight = q_len;
            selected_min_path_hop_count = paths[ii].size()-1;
        }
    }
    
    if (flag){
        cout << "non-min paths q len:" << endl;
    }    
    //go through the non-min paths and select the one with the lowest Q length
    for(ii = no_of_min_paths_to_consider; ii < (no_of_min_paths_to_consider + no_of_VLB_paths_to_consider); ii++){
        q_len = find_port_queue_len_to_node(r, paths[ii][0], paths[ii][1]);
        if (flag){
            cout << "path " << ii << " q_len: " << q_len << endl;
        }
        if (q_len < min_VLB_path_weight){
            selected_VLB_path_id = ii;
            min_VLB_path_weight = q_len;
            selected_VLB_path_hop_count = paths[ii].size()-1;
        }
    }
    
    
    if (g_ugal_multiply_mode == "one_vs_two"){
        min_multiplier = 1;
        vlb_multiplier = 2;
    }else if(g_ugal_multiply_mode == "pathlen_based"){
        min_multiplier = selected_min_path_hop_count;
        vlb_multiplier = selected_VLB_path_hop_count;
    } else{
        cout << "Erorr! Unsupported ugal_multiply_mode: " << g_ugal_multiply_mode << " . Exiting." << endl;
        exit(-1);
    }

    //make comparison
    if ( (min_shortest_path_weight * min_multiplier) <= (min_VLB_path_weight * vlb_multiplier) ){
        chosen_path_id = selected_min_path_id;
        
        chosen  = 0;
        g_total_min_flit += 1;
        g_cases_when_not_taken[chosen_tier] += 1;
        g_lens_when_not_taken[selected_VLB_path_hop_count] += 1;
        
        if (src_group == dst_group){
            g_min_ingroup_path_dist[selected_min_path_hop_count] += 1;
        }else{
            g_min_outgroup_path_dist[selected_min_path_hop_count] += 1;
        }

        if (flag){
            cout << "choosing MIN path. path id: " << chosen_path_id << " pathlen: " << selected_min_path_hop_count << endl;
        }        

    }
    else{
        chosen_path_id = selected_VLB_path_id;
        
        chosen = 1;
        g_total_non_min_flit += 1;
        g_cases_when_taken[chosen_tier] += 1;
        g_lens_when_taken[selected_VLB_path_hop_count] += 1;

        /*if ((selected_VLB_path_hop_count == 4) && (chosen_tier == 4)){
            cout << "flit " << f->id << " : ";
            for(int ii = 0; ii < paths[chosen_path_id].size(); ii++){
                cout << paths[chosen_path_id][ii] << "  ";
            }            
            cout << endl;
        }*/

        if (src_group == dst_group){
            g_vlb_ingroup_path_dist[selected_VLB_path_hop_count] += 1;
        }else{
            g_vlb_outgroup_path_dist[selected_VLB_path_hop_count] += 1;
        }


        if (flag){
            cout << "choosing VLB path. path id: "  << chosen_path_id << " pathlen: " << selected_VLB_path_hop_count <<  " flit id:" << f->id << endl;
        }
    }

    log_ugal_stats(chosen, min_multiplier, vlb_multiplier);
    
    string q_len_data = "";

    if (g_log_Qlen_data == 1){
        if (chosen == 1){
            //cout << "Router:, " << r->GetID() << " , " << "min_q:, " << min_shortest_path_weight << " , " << "non-min_q:, " << min_VLB_path_weight << " , chosen:, " << chosen << endl;
        //return the id of the chosen one
            q_len_data += std::to_string(r->GetID());
            q_len_data += ",";
            q_len_data += std::to_string(min_shortest_path_weight);
            q_len_data += ",";
            q_len_data += std::to_string(min_VLB_path_weight);
            q_len_data += ",";
            q_len_data += std::to_string(chosen);
            q_len_data += "\n";
            
            g_q_len_file << q_len_data;
        }
    }

    return chosen_path_id;
}

int make_UGAL_G_path_choice(const Router *r, const Flit *f, int no_of_min_paths_to_consider, int no_of_VLB_paths_to_consider, std::vector< std::vector<int> > paths, int chosen_tier){
    /*
    chosen_tier is only here for some legacy stat-collection code. Not required
    at all.
    */
    
    bool flag = false;

    int min_shortest_path_weight = 9999; //arbitrary large number
    int min_VLB_path_weight = 9999;
    
    int ii, jj;
    int q_len_sum;
    int temp_q_len;
    
    int selected_min_path_id = -1;  //dummy values. If they are not get changed then it means we have a problem.
    int selected_VLB_path_id = -1;    
    int chosen_path_id;

    int selected_min_path_hop_count = 0;
    int selected_VLB_path_hop_count = 0;

    int chosen;

    //src_group and dst_group are only needed here for stat collection.
    int src_group = ((f->src) / g_p ) / g_a; 
    int dst_group = ((f->dest) / g_p ) / g_a; 


    //go through the min paths and select the one with the lowest Q length sum
    if (flag){
        cout << "min paths q len:" << endl;
    }
    for(ii = 0; ii< no_of_min_paths_to_consider; ii++){
        q_len_sum = 0;
        
        for(jj = 0; jj < paths[ii].size() - 1; jj++){
            q_len_sum  += find_port_queue_len_to_node(all_routers[paths[ii][jj]], paths[ii][jj], paths[ii][jj+1]);
        }
        if (flag){
            cout << "path " << ii << " q_len_sum: " << q_len_sum << endl;
        }

        if (q_len_sum < min_shortest_path_weight){
            selected_min_path_id = ii;
            min_shortest_path_weight = q_len_sum;
            selected_min_path_hop_count = paths[ii].size()-1;
        }
    }

    //go through the vlb paths and select the one with the lowest Q length sum
    if (flag){
        cout << "non-min paths q len:" << endl;
    }    
    //go through the non-min paths and select the one with the lowest Q length
    for(ii = no_of_min_paths_to_consider; ii < (no_of_min_paths_to_consider + no_of_VLB_paths_to_consider); ii++){
        
        q_len_sum = 0;
        for(jj = 0; jj < paths[ii].size() - 1; jj++){
            q_len_sum  += find_port_queue_len_to_node(all_routers[paths[ii][jj]], paths[ii][jj], paths[ii][jj+1]);
        }

        if (flag){
            cout << "path " << ii << " q_len_sum: " << q_len_sum << endl;
        }

        if (q_len_sum < min_VLB_path_weight){
            selected_VLB_path_id = ii;
            min_VLB_path_weight = q_len_sum;
            selected_VLB_path_hop_count = paths[ii].size()-1;
        }
    }
    


   
    //make comparison
    if ( min_shortest_path_weight  <= min_VLB_path_weight ){
        chosen_path_id = selected_min_path_id;
        
        chosen  = 0;
        g_total_min_flit += 1;
        g_cases_when_not_taken[chosen_tier] += 1;
        g_lens_when_not_taken[selected_VLB_path_hop_count] += 1;
        
        if (src_group == dst_group){
            g_min_ingroup_path_dist[selected_min_path_hop_count] += 1;
        }else{
            g_min_outgroup_path_dist[selected_min_path_hop_count] += 1;
        }

        if (flag){
            cout << "choosing MIN path. path id: " << chosen_path_id << " pathlen: " << selected_min_path_hop_count << endl;
        }        

    }
    else{
        chosen_path_id = selected_VLB_path_id;
        
        chosen = 1;
        g_total_non_min_flit += 1;
        g_cases_when_taken[chosen_tier] += 1;
        g_lens_when_taken[selected_VLB_path_hop_count] += 1;

        if (src_group == dst_group){
            g_vlb_ingroup_path_dist[selected_VLB_path_hop_count] += 1;
        }else{
            g_vlb_outgroup_path_dist[selected_VLB_path_hop_count] += 1;
        }


        if (flag){
            cout << "choosing VLB path. path id: "  << chosen_path_id << " pathlen: " << selected_VLB_path_hop_count<< endl;
        }
    }

    return chosen_path_id;

}

void log_ugal_stats(int min_or_vlb_choice, int min_multiplier, int vlb_multiplier){
    //for stat collection

    auto pathlen_pair = std::make_pair(min_multiplier, vlb_multiplier);

    if ( g_pathlen_pair_freq_for_ugal_comparison.find( pathlen_pair ) != g_pathlen_pair_freq_for_ugal_comparison.end() ){
        g_pathlen_pair_freq_for_ugal_comparison[pathlen_pair].tot += 1;
    }else{
        struct Counters val;
        g_pathlen_pair_freq_for_ugal_comparison[pathlen_pair] = val;
    }
    if (min_or_vlb_choice == 0){
        //min chosen
        g_pathlen_pair_freq_for_ugal_comparison[pathlen_pair].min += 1;   
    }else{
        //vlb chosen
        g_pathlen_pair_freq_for_ugal_comparison[pathlen_pair].vlb += 1;   
    }

}


int find_port_queue_len_to_node(const Router *r, int current_router, int next_router){
    //first find the port connected to the next_router
    
    /*
    Issue: 
    Though unlikely, but theoretically it is possible for some arrangements that
    some nodes will have multiple links between them. In that case how do we 
    take Q len or select a node?
    Well, the situation is not unlike SlimFly b/w allocation scheme.
    So we can just do the same and take the average of the Q_lens.
    While forwarding a node, we can select a Q randomly.     
    */
    
    auto port_range = g_port_map[std::make_pair(current_router, next_router)];
    
    //then check its Q_length
    int port_id;
    
    int queue_length = 0;
    int count = 0;
        
    for (port_id = port_range.first; port_id <= port_range.second; port_id++) {
        //port_range is a closed range
        queue_length += r->GetUsedCredit(port_id);
        count += 1;
    }
    
    queue_length = queue_length / count; 
        /* 
        //        This can result in some loss of fraction. 
        //        A more perfect way is probably to have it as a float. May be later. 
        */                             
    
    return queue_length;
}
