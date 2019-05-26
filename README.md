The files included here are:

├── Booksim_Topology_And_Routing
│   ├── booksim_config.cpp
│   ├── djkstra.cpp
│   ├── djkstra.hpp
│   ├── dragonfly_full.cpp
│   ├── dragonfly_full.hpp
│   └── pair_hash.hpp
├── LinearModleing
│   └── mcf.py
└── README.txt


The purpose of the code shared here is to help reproducing the results that were included
in our paper submitted to SC 2019, "Topology-Custom UGAL Routing on Dragonfly".

Please note, that this is not the complete, running code-base. Rather it just includes 
code to create the topology and routing functions in Booksim, and to create linear 
programming models which can be solved by IBM CPLEX or something similar. 

For Booksim, download Booksim 2.0. Then put the included source files in the src/networks
directory. It should be good to go. The compiler may complain with some code that 
were included for debugging/stat-collection purpose, just deactivate those. 

For Linear Modeling, mcf.py should be enough to understand the basics of the model. 
It is a modifiled version of model 3 described in "Modeling ugal on the dragonfly
topology" by Mollah et al, check that for a more thorough understanding. 
The code expects a topology object and the list of shortest-paths to
be passed. It should be fairly easy to generate them following the topology
and djkstra path generation source-code included for Booksim.

The code is provided as-is and no support is guaranteed. 

All rights reserved by FSU CS EXPLORER lab (https://explorer.cs.fsu.edu).

Shafayat Rahman
25 May 2019
