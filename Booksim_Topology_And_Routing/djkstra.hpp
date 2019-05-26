void generate_path(int src, int dst, std::vector< std::vector< std::vector<int> > >& parents, std::vector< std::vector<int> >& final_paths);

void generate_path_internal(int current_node, int dst, std::vector<int> & current_path, std::vector<std::vector<int> >& final_path, std::vector< std::vector< std::vector<int> > >& parents);

void djkstra(int src, int N, std::vector < std::vector < std::pair<int,int> > >& graph, std::vector< int >& distance, std::vector< std::vector<int> >& parents);

void all_pair_djkstra(int N, std::vector < std::vector < std::pair<int,int> > >& graph, std::vector< std::vector< int >>& distance, std::vector< std::vector< std::vector<int> > >& parents);
