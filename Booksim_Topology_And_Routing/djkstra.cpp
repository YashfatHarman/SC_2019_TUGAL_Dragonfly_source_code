/*
Implement a djkstra that we can use in booksim routing.

The skeleton code is already in python, let's translate that first.
Will worry about the interfaces later.
*/

/*
Data type:
    - We are not saving all the paths. Rather it'll be just the parent tables 
    from which paths can be generated on the fly.
    - For each src, we need to consider parent for each destination. So a 2D vector
    of N*N dimension.
    - Because we consider multiple paths, each cell needs to be a vector as well.
    - So a 3d vector of N*N*<var>
*/

#include <iostream>
#include <vector>
#include <queue>

#include "djkstra.hpp"

#define INF 9999

using namespace std;

std::vector < std::vector <std::vector <int> > > parents;
//std::vector < std::vector <int>  > parents;
int N = 5;

void d3_vector_test();
void check_djkstra();


void d3_vector_test(){
    /*
    Just a simple test to see how to work with a 3D vector, with the 
    3rd dimension of variable length.
    */
    parents.resize(N, std::vector< std::vector<int> > (N, std::vector<int>() ) );
    cout << "vector resized" << endl;
    
    //populate vector
    int val;
    for( std::size_t ii = 0; ii<parents.size(); ii++){
        for( std::size_t jj = 0; jj<parents[ii].size(); jj++){
            for ( std::size_t kk =0; kk < ii+jj+1; kk++){
                val = ii * 100 + jj * 10 + kk;
                parents[ii][jj].push_back(val);
            }
        }
    }
    cout << "population done." << endl;
    //print contents
    for( std::size_t ii = 0; ii<parents.size(); ii++){
        for( std::size_t jj = 0; jj<parents[ii].size(); jj++){
            cout << "(" << ii << "," << jj << ") :" ;
            for ( std::size_t kk =0; kk < parents[ii][jj].size(); kk++){
                cout << parents[ii][jj][kk] << " ";
            }
            cout << endl;
        }
    }
}

class MyComparison{
    /*  A comparison function object that compares two pairs and
        chooses the one with the smaller second element as the winner.*/
    public:
        MyComparison(){
        }
        
        bool operator() (const std::pair<int,int>& lhs, const std::pair<int,int>& rhs) const{
            return (lhs.second > rhs.second);
        }

};

void djkstra(int src, int N, std::vector < std::vector < std::pair<int,int> > >& graph, std::vector< int >& distance, std::vector< std::vector<int> >& parents){
    /*
    Finds all wieghted shortest paths from src.
    Will populate the Parents list. Will think about its structure later.
    
    src:    the node from where the paths are started
    N:      total nodes in the network
    graph:  Adjacency list along with path weight.
            For each src node, there will be a list of (dst, link_weight)
    */
    
    //    cout << "inside djkstra ..." << endl;
    //    cout << "src: " << src << endl;
    //    cout << "N: " << N << endl;
        
    //std::vector< int > distance(N, INF);
    //std::vector< std::vector<int> > parents(N);
    std::vector< bool > visited(N, false);    
    
    std::priority_queue< std::pair<int,int>, vector<std::pair<int,int>>, MyComparison > min_heap;
    
    std::pair<int,int> top_pair;
    //int top_distance;
    int top_node;
    int neighbor_weight, neighbor_node;
    
    //actual algorithm starts. put the top node in heap.
    distance[src] = 0;
    parents[src].push_back(-1); //-1 will work as a path terminator
    
    min_heap.push(std::make_pair(src,distance[src]));    
        //Unlike the python version, here the comparison function in the heap actually
        //works with the second element. So have the distance as the second element.
        
    //while the heap is empty
    while(!min_heap.empty()){
        //pop the top node, the one with minimum weight
        top_pair = min_heap.top();
        top_node = top_pair.first;
        //top_distance = top_pair.second;
        
        min_heap.pop();
        
        //mark it as visited
        visited[top_node] = true;
        
        //check each neighbor of the top_node
        for( std::size_t ii = 0; ii < graph[top_node].size(); ii++){
            neighbor_node = graph[top_node][ii].first;
            neighbor_weight = graph[top_node][ii].second;
            
            //if neighbor is not already visited:
            if (visited[neighbor_node] == false){
                //weight checking case 1: smaller path
                if ( (distance[top_node] + neighbor_weight) < distance[neighbor_node]) {
                    distance[neighbor_node] = distance[top_node] + neighbor_weight;
                    parents[neighbor_node] = std::vector<int>{top_node};
                    min_heap.push(std::make_pair(neighbor_node, distance[neighbor_node]));
                } 
                //weight checking case 2: equal path     
                else if( (distance[top_node] + neighbor_weight) == distance[neighbor_node]){
                    distance[neighbor_node] = distance[top_node] + neighbor_weight;
                    parents[neighbor_node].push_back(top_node);
                    min_heap.push(std::make_pair(neighbor_node, distance[neighbor_node]));
                }
            }
        }
        
    }//that should be it    
    
    //cout << "end of djkstra." << endl;
    
    /*cout << "Test printing the distances table " << endl;
    for(int ii = 0; ii<distance.size(); ii++){
        cout << ii << "," << distance[ii] << endl;
    }
    
    cout << "\ntest printing the parents table:" << endl;
    for(int ii=0; ii<parents.size(); ii++){
        cout << ii << " : ";
        for(int jj=0; jj < parents[ii].size(); jj++){
            cout << parents[ii][jj] << " ";
        }
        cout << endl;
    }*/
    
}

void all_pair_djkstra(int N, std::vector < std::vector < std::pair<int,int> > >& graph, std::vector< std::vector< int >>& distance, std::vector< std::vector< std::vector<int> > >& parents){
    /*
    For every source node in the graph, call djksta.
    */
    
    /*
    N: total nodes
    graph: weighted adjacency list
    distance: 2D vector containing the djksta distance between each SD pair
    parents: 3d vector containing the parent of each node according to djkstra
    */
    
    int src;
    
    for(src = 0; src<N; src++){
        djkstra(src, N, graph, distance[src], parents[src]);
        
        /*cout << "Test printing the distances table " << endl;
        for( int ii = 0; ii<distance[src].size(); ii++){
            cout << ii << "," << distance[src][ii] << endl;
        }*/
        
        
    }
    
    /*cout << "\ntest printing the parents table:" << endl;
    for(src = 0; src < N; src++){
        cout << "src " << src << " -> "; 
        for( int ii=0; ii<parents[src].size(); ii++){
            cout << " (" << ii << " : ";
            for( int jj=0; jj < parents[src][ii].size(); jj++){
                cout << parents[src][ii][jj] << ",";
            }
            cout << ") ";
        }
        cout << endl;
    }*/
    
    /*src = 19;
    int dst = 0;
    cout << "calling generate path on " << src << " and " << dst << endl;
    generate_path(src, dst, parents);
    cout << "path generation returned" << endl;*/
}


void generate_path(int src, int dst, std::vector< std::vector< std::vector<int> > >& parents, std::vector< std::vector<int> >& final_paths){
    /*
    Given that a parent list is alreayd generated, this function
    generates the path from a given source to a given destination.
    
    src, dst: as they sounds.
    parents: data structure generated by djkstra.
    */
    bool flag = false;

    int src_node = src;
    int current = dst;
    
    std::vector<int> current_path;
    //std::vector< std::vector<int> > final_paths;
    
    generate_path_internal(src_node, current, current_path, final_paths, parents);
    
    //    cout << "\nBack to outer function: " << endl;
    if (flag){
        cout << "src node: " << src_node << endl;
        cout << "current: " << current << endl;
        cout << "current path: ";
        for(auto it = current_path.begin(); it!= current_path.end(); it++){
            cout << *it << " ";
        }
        cout << endl;
        
        cout << "total path found: " << final_paths.size() << endl;
        for( std::size_t ii = 0; ii < final_paths.size(); ii++){
            cout << "path " << ii << " : ";
            for( std::size_t jj = 0; jj < final_paths[ii].size(); jj++ ){
                cout << final_paths[ii][jj] << " ";
            }
            cout << endl;
        }
        cout << endl;
    }
}

void generate_path_internal(int src_node, int current, std::vector<int>&  current_path, std::vector<std::vector<int> >& final_path, std::vector< std::vector< std::vector<int> > >& parents){
    /*
    Recursive function to generate all the paths between a src and dst. 
    Called by generate_path() from outside.
    
    src_node: the src of the path. The way this function works, it starts from dest and
            comes inward until the src_node is found. That is why this needs to be passed
            around to know when the terminating condition is found.
    current: current node. This is added to the path, and the function 
            is called recursively over its parent(s)
    current_path: running path. Starts with only the dest, and added one node at a time
                until the src is found.
    final_path: 2D vector to hold all the paths. Whenever a path is found, 
                it is appended to this. 
    parents: data structure, populated by djkstra. Basically keep the predecessor node 
            determined through djkstra algorithm.
    
    */

    /* This one is the actual recursive function called by the generate_path() */
    
    //    cout << "src node: " << src_node << endl;
    //    cout << "current : " << current << endl;
    //    cout << "parents: ";
    //    for(size_t ii = 0; ii < parents[src_node][current].size(); ii++){
    //        cout << parents[src_node][current][ii] << " ";
    //    }
    //    cout << endl;

    current_path.push_back(current);
    
    //    cout << "current path:" ;
    //    for(size_t ii = 0; ii < current_path.size(); ii++){
    //        cout << current_path[ii] << " ";
    //    }
    //    cout << endl;
    //    
    
     int parent_count = parents[src_node][current].size();
    //cout << "parent_count: " << parent_count << endl;
    
    int parent;

    if (parent_count == 1){
            parent = parents[src_node][current][0];
            
            if (parent == -1){
                //path found. save final result.
                //cout << "branch 1. Path found." << endl;
                std::vector<int> temp_path(current_path.size(), -1);
                int jj, kk;
                for(jj = current_path.size() - 1, kk = 0; jj >= 0 ; jj--, kk++){
                    //we generated the path from dest to src, so reversing it
                    //cout << jj << "," << kk << endl;
                    temp_path[kk] = current_path[jj];
                }
                final_path.push_back(temp_path);
                //cout << "branch 1. Path found. End branch." << endl;
                
            }else{
                //path not found yet. dig deeper.
                //cout << "branch 1. Path Not found." << endl;
                
                //no branching. No need to copy the path. Pass by reference.
                generate_path_internal(src_node, parent, current_path, final_path, parents);
            }
        
    }else{
                
        for ( int ii = 0; ii < parent_count; ii++){
            parent = parents[src_node][current][ii];
            //cout << "branch 2 for parent " << parent << endl;
        
            if (parent == -1){
                //path found. save final result.
                //cout << "branch 2. Path found." << endl;
        
                std::vector<int> temp_path(current_path.size(), -1);
                int jj, kk;
                for(jj = current_path.size() - 1, kk = 0; jj >= 0 ; jj--, kk++){
                    //we generated the path from dest to src, so reversing it
                    temp_path[kk] = current_path[jj];
                }
                final_path.push_back(temp_path);
                //cout << "branch 2. Path found. End branch." << endl;
                
            }else{
                //path not found yet. dig deeper.
                
                //branching. So make a new copy of current_path
                //cout << "branch 2. Path Not found." << endl;
                std::vector<int>  current_path_copy(current_path);
                //cout << "branch 2. Path Not found. Path copied." << endl;
                
                generate_path_internal(src_node, parent, current_path_copy, final_path, parents);
            }
        }
        
    }
}

void check_djkstra(){
    //genreate the graph as per function requirement
        //test network (DF of a=2, g=3)
        //        0 [(1, 1), (2, 3)]
        //        1 [(0, 1), (4, 3)]
        //        2 [(0, 3), (3, 1)]
        //        3 [(2, 1), (5, 3)]
        //        4 [(1, 3), (5, 1)]
        //        5 [(3, 3), (4, 1)]
    
    int N = 4;
    std::vector < std::vector < std::pair<int,int> > > graph(N);
    
    graph[0].push_back(std::make_pair(1,1));
    graph[0].push_back(std::make_pair(2,3));
    
    graph[1].push_back(std::make_pair(0,1));
    graph[1].push_back(std::make_pair(3,3));
    
    graph[2].push_back(std::make_pair(0,3));
    graph[2].push_back(std::make_pair(3,1));
    
    graph[3].push_back(std::make_pair(1,3));
    graph[3].push_back(std::make_pair(2,1));
    
    /*graph[4].push_back(std::make_pair(1,3));
    graph[4].push_back(std::make_pair(5,1));
    
    graph[5].push_back(std::make_pair(3,3));
    graph[5].push_back(std::make_pair(4,1));
    */
    
    cout << "list of link generated: " << endl;
    for( std::size_t ii = 0; ii<graph.size(); ii++){
        for ( std::size_t jj = 0; jj<graph[ii].size(); jj++){
            cout << ii << " , " << graph[ii][jj].first << " , " << graph[ii][jj].second << endl; 
        }
    }
    
    std::vector< std::vector< int >> distance(N, std::vector< int >(N,INF));
    std::vector< std::vector< std::vector<int> > > parents(N, std::vector< std::vector<int> >(N));
    
    all_pair_djkstra(N, graph, distance, parents);
    
    //call djkstra
    /*djkstra(0, 4, graph);
    djkstra(1, 4, graph);
    djkstra(2, 4, graph);
    djkstra(3, 4, graph);*/
}

int main_1(){
    cout << "Hello World!" << endl;
    
    check_djkstra();
    
    cout << "Bye.Go die!" << endl;
    return 0;
}



























