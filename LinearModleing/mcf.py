'''
Initial target: To implement a function to generate MCF rules for a given topology and routing.
                The MCF rules can then be solved using cplex or similar.
                
Later we may extend it to include other throghput metrics.
'''

import networkx as nx
import os
import Dragonfly_paths as DP
import sys
import random
import math

'''
Workflow:
    
    Variables:
        For each flow:
            For each path-type (MIN/ VLB):
                For each path len:
                    Make a virable X_sd_type_len
    
    Gather info:
        
        For each SD pair, we need:
            - no. of MIN paths of each length  # For DF it can be of 1-3 hops
            - no. of VLB paths of each length  # For DF it can of 2-6 hops
    
        For each SD pair,
            For each type of paths (MIN / VLB)
                For each path len
                    For each link:    
                        Keep track of the (variable, coefficient) pair for the link.
                            # we don't need any coefficients as the variables are for flow rate on
                            a single link of a certain type. So directly putting the variables 
                            in the link equations is fine.
                            
            For the source:
                Keep track of the varible X_sd_type_len
                
            For the destination:
                Keep track of the variable X_sd_type_len
                    
    
    Data Structure needed:
        
        For the flows a dict of dict: <flow, <dict2>>
                                dict2: <(pathtype, pathlen), count>
        
        For the links, a dict of dict: < (link), <dict2> >
                                dict2: < (flow, type, len), coefficient>
                                
        For the source routers: a dict of dict: <src, <dict2>>
                                dict2: < (flow, type, len), count>
        
        For the destination routers: a dict of dict: <dst, <dict2>>
                                dict2: < (flow, type, len), count>
                             
    Generate rules:
        
        For each flow:
            sum of ( type_len_count * X_flow_type_len) <= 1
            
        For each src:
            sum of (X_flow_type_len) <= 1
            
        For each dst:
            sum of (X_flow_type_len) <= 1

        For each link:
            sum of (X_flow_type_len) <= 1
'''

def dict_insert(element, dict):
    if element in dict:
        dict[element] += 1
    else:
        dict[element] = 1

def get_min_paths_count__pathlen_based_control(**kwargs):
    '''
    pathlen_based_control => a variable for each path length (in mcf)
    
    sd_pairs : a list of <src_PE, dst_PE> tuples.
    '''
    
    min_pathlist = kwargs.get("min_pathlist")
    sd_pairs = kwargs.get("sd_pairs")
    N = kwargs.get("N")
    p = kwargs.get("p")
    
    min_paths_count = {}
    
    for src, dst in sd_pairs:
        #print(src,dst)
        if src == dst:
            continue
        
        src_router = src // p
        dst_router = dst // p
        
        if src_router != dst_router:
            #first deal with all the min paths
            paths = min_pathlist[src_router][dst_router]
            #print(paths)
            
            temp_dict = {}
            for path in paths:
                pathlen = len(path) - 1 + 2 
                    # path contains start and end nodes, so no of hop is one less than no of nodes.
                    # +2 to include the src_PE and dst_PE into the path consideration as well. 
                    
                dict_insert(pathlen, temp_dict)
                
            min_paths_count[(src, dst)] = sorted(temp_dict.items())
            
        else:
            #src and dst are under the same router
            min_paths_count[(src,dst)] = [(2,1)]    #only 1 path of len 2
        
    return min_paths_count


def get_min_paths_count__all_random_control(min_pathlist, sd_pairs, *, N, p):
    '''
    all_random_control => only one variable for all the min paths per SD pair
    So return only the count of min paths per each pair.
    
    sd_pairs : a list of <src_PE, dst_PE> tuples.
    '''
    
    min_paths_count = {}
    
    for src, dst in sd_pairs:
        #print(src,dst)
        if src == dst:
            continue
        
        src_router = src // p
        dst_router = dst // p
        
        if src_router != dst_router:
            #first deal with all the min paths
            paths = min_pathlist[src_router][dst_router]
            #print(paths)
            
            min_paths_count[(src, dst)] = [(0, len(paths))]
                    # 0 is default length. Total no of paths is the only thing that matters here.
            
        else:
            #src and dst are under the same router
            min_paths_count[(src,dst)] = [(0,1)]    
                #only 1 path of len 0. Len 0 is the default len in this mode.
        
    return min_paths_count

'''
*** Important: Signature of all the inode_generator functions need to be exact same.
'''

#def inode_generator_empty(src_router, dst_router, N, a, *, twoHopNeighborList = None, sd_pair_vs_4hop_inodes = None, sd_pair_vs_3hop_inodes = None,  mode_param = 0):
def inode_generator_empty(src_router, dst_router, **kwargs):
    '''
    This is basically a dummy placeholder function to be used where we do not
    want to use any VLB path, rather will use min paths only.
    Returns an empty list.
    '''
    
    return []
    pass

#def inode_generator_vanilla(src_router, dst_router, N, a, *, twoHopNeighborList = None, sd_pair_vs_4hop_inodes = None,sd_pair_vs_3hop_inodes = None, mode_param = 0):
def inode_generator_vanilla(src_router, dst_router, **kwargs):
    '''
    src and dst are routers.
    
    twoHopNeighborList is not needed here at all. This is only provided
    to keep the function signature same for each inode_generator functions. 
    '''
    
    N = kwargs.get("N")
    a = kwargs.get("a")
    
    #if src and dst are in differnt group, generate every other node outside src 
    #and dst's groups as potential inodes
    
    #else, get only the other nodes in the same group as src and dst as the inodes.
    
    src_group = src_router // a
    dst_group = dst_router // a
    
    if src_group == dst_group:
        lst = [x for x in range(src_group*a , src_group*a + a) if x != src_router and x!= dst_router]
    else:
        lst = [x for x in range(N) if x//a != src_group and x//a != dst_group]
    
    
    return lst
    
#def inode_generator_2hop_src_and_dst(src_router, dst_router, N, a, *, twoHopNeighborList, sd_pair_vs_4hop_inodes = None, sd_pair_vs_3hop_inodes = None,mode_param = 0):
def inode_generator_5hop_src_and_dst(src_router, dst_router, **kwargs):
    '''
    For restricted vlb / ugal routing in DF.
    
    For an SD pair in a DF, only return the nodes that can n a maximum of 2 hops
    from either the src or the dst.
    
    '''
    
    '''
    Algo: we need the list of all global links in the DF.
    
    For an SD pair, return the union of src's and dst's 2hop neighbors.
    '''
    
    a = kwargs.get("a")
    twoHopNeighborList = kwargs.get("twoHopNeighborList")
    
    src_2hop_neighbors = set([x for x in twoHopNeighborList[src_router] if dst_router//a != x//a])
    dst_2hop_neighbors = set([x for x in twoHopNeighborList[dst_router] if src_router//a != x//a])
    
    return list(src_2hop_neighbors | dst_2hop_neighbors)

    pass
    
#def inode_generator_2hop_src_only(src_router, dst_router, N, a, *, twoHopNeighborList, sd_pair_vs_4hop_inodes = None, sd_pair_vs_3hop_inodes = None, mode_param = 0):
def inode_generator_5hop_src_only(src_router, dst_router, **kwargs):
    '''
    For restricted vlb / ugal routing in DF.
    
    For an SD pair in a DF, only return the nodes that can n a maximum of 2 hops
    from either the src.
    
    In simulation, we found that inode_generator_2hop_src_only works better
    than inode_generator_2hop_src_and_dst. 
    
    '''
    a = kwargs.get("a")
    twoHopNeighborList = kwargs.get("twoHopNeighborList")
    
    return [x for x in twoHopNeighborList[src_router] if dst_router//a != x//a] #ignoring the neighbors that belong to src's group

    pass

#def inode_generator_for_4hop_paths(src_router, dst_router, N, a, *, twoHopNeighborList = None, sd_pair_vs_4hop_inodes, sd_pair_vs_3hop_inodes = None, mode_param = 0):
def inode_generator_for_4hop_paths(src_router, dst_router, **kwargs):    
    '''
    twoHopNeighborList and sd_pair_vs_4hop_inodes are not needed. They are here just to match the function signature.
    '''
    sd_pair_vs_4hop_inodes = kwargs.get("sd_pair_vs_4hop_inodes")
    
    return sd_pair_vs_4hop_inodes[(src_router, dst_router)]
    
    pass

#def inode_generator_for_3hop_paths(src_router, dst_router, N, a, *, twoHopNeighborList = None, sd_pair_vs_4hop_inodes = None, sd_pair_vs_3hop_inodes, mode_param = 0):
def inode_generator_for_3hop_paths(src_router, dst_router, **kwargs):    
    '''
    twoHopNeighborList and sd_pair_vs_4hop_inodes are not needed. They are here just to match the function signature.
    '''
    sd_pair_vs_3hop_inodes = kwargs.get("sd_pair_vs_3hop_inodes")
    
    return sd_pair_vs_3hop_inodes[(src_router, dst_router)]
    
    pass

#def inode_generator_3hop_and_4hop(src_router, dst_router, N, a, *, twoHopNeighborList = None, sd_pair_vs_4hop_inodes, sd_pair_vs_3hop_inodes, mode_param):
def inode_generator_3hop_and_4hop(src_router, dst_router, **kwargs):
    '''
    Get all the 3 hop inodes.
    Get all the 4 hop inodes.
    Get the extra 4-hop inodes.
    Get a portion of them.
    Return the 3 hop + the portion of extra 4 hops.
    '''
    sd_pair_vs_3hop_inodes = kwargs.get("sd_pair_vs_3hop_inodes")
    sd_pair_vs_4hop_inodes = kwargs.get("sd_pair_vs_4hop_inodes")
    mode_param = kwargs.get("mode_param")
    
    
    threeHopNodes = set(sd_pair_vs_3hop_inodes[(src_router, dst_router)])
    #print("threeHopNodes", threeHopNodes)
    
    fourHopNodes = set(sd_pair_vs_4hop_inodes[(src_router, dst_router)])
    #print("fourHopNodes", fourHopNodes)
    
    extra_4_hopNodes = list(fourHopNodes - threeHopNodes)
    #print("extra_4_hopNodes", extra_4_hopNodes)
    
    random.shuffle(extra_4_hopNodes)
    #print("extra_4_hopNodes after shuffle", extra_4_hopNodes)
    
    cut_off_point = math.floor(len(extra_4_hopNodes)*mode_param/100)
    #print("cut_off_point:",cut_off_point)
    
    eligible_inodes = list(threeHopNodes)
    #print("eligible_inodes:",eligible_inodes)
    
    eligible_inodes.extend(extra_4_hopNodes[:cut_off_point+1])
    #print("eligible_inodes:",eligible_inodes)
    
    return eligible_inodes
    
    pass


#def inode_generator_4hop_and_5hop(src_router, dst_router, N, a, *, twoHopNeighborList, sd_pair_vs_4hop_inodes, sd_pair_vs_3hop_inodes = None, mode_param):
def inode_generator_4hop_and_5hop(src_router, dst_router, **kwargs):
    
    '''
    Generate all 4 hop paths. 
    Generate all 5 hop paths.
    Take a chuck of 5 hop paths.
    Return the list of 4 hop paths plus the chunk.
    '''
    a = kwargs.get("a")
    twoHopNeighborList = kwargs.get("twoHopNeighborList")
    sd_pair_vs_4hop_inodes = kwargs.get("sd_pair_vs_4hop_inodes")
    mode_param = kwargs.get("mode_param")
    
    #src_group = src_router // a
    #dst_group = dst_router // a
    #print("src_router,dst_router,src_group, dst_group", src_router,dst_router,src_group, dst_group)
    
    fourHopInodes = set(sd_pair_vs_4hop_inodes[(src_router,dst_router)])
    #print("fourHopInodes:", fourHopInodes)
    
    src_2hop_neighbors = set([x for x in twoHopNeighborList[src_router] if dst_router//a != x//a])
    #print("src_2hop_neighbors:",src_2hop_neighbors)
    dst_2hop_neighbors = set([x for x in twoHopNeighborList[dst_router] if src_router//a != x//a])
    #print("dst_2hop_neighbors:", dst_2hop_neighbors)
    
    eligible_5hop_inodes = src_2hop_neighbors | dst_2hop_neighbors
    #print("eligible_5hop_inodes:",eligible_5hop_inodes)
    
    extra_inodes = list(eligible_5hop_inodes - fourHopInodes)
    #print("extra_inodes:", extra_inodes)
    
    #shuffle it, take first mode_param%
    random.shuffle(extra_inodes)
    #print("extra_inodes shuffled:", extra_inodes)
    
    cut_off_point = math.floor(len(extra_inodes)*mode_param/100)
    #print("cut_off_point:",cut_off_point)
    
    eligible_inodes = list(fourHopInodes)
    #print("eligible_inodes:",eligible_inodes)
    
    eligible_inodes.extend(extra_inodes[:cut_off_point+1])
    #print("eligible_inodes:",eligible_inodes)
    
    return eligible_inodes

    
    pass


#def inode_generator_5hop_and_6hop(src_router, dst_router, N, a, *, twoHopNeighborList, sd_pair_vs_4hop_inodes = None , sd_pair_vs_3hop_inodes = None, mode_param):
def inode_generator_5hop_and_6hop(src_router, dst_router, **kwargs):
    '''
    Take all 5hop paths. Then take mode_param% of remaining nodes.
    '''
    a = kwargs.get("a")
    N = kwargs.get("N")
    twoHopNeighborList = kwargs.get("twoHopNeighborList")
    mode_param = kwargs.get("mode_param")
    
    src_group = src_router // a
    dst_group = dst_router // a
    
    #print("src,dst,src_group, dst_group", src,dst,src_group, dst_group)
    
    src_2hop_neighbors = set([x for x in twoHopNeighborList[src_router] if dst_router//a != x//a])
    #print("src_2hop_neighbors:",src_2hop_neighbors)
    dst_2hop_neighbors = set([x for x in twoHopNeighborList[dst_router] if src_router//a != x//a])
    #print("dst_2hop_neighbors:", dst_2hop_neighbors)
    
    eligible_5hop_inodes = src_2hop_neighbors | dst_2hop_neighbors
    #print("eligible_5hop_inodes:",eligible_5hop_inodes)
    
    all_inodes = set([x for x in range(N) if x//a != src_group and x//a != dst_group])
    #print("all_inodes:", all_inodes)
    
    extra_inodes = list(all_inodes - eligible_5hop_inodes)
    #print("extra_inodes:", extra_inodes)
    
    #shuffle it, take first mode_param%
    random.shuffle(extra_inodes)
    #print("extra_inodes shuffled:", extra_inodes)
    
    cut_off_point = math.floor(len(extra_inodes)*mode_param/100)
    #print("cut_off_point:",cut_off_point)
    
    eligible_inodes = list(eligible_5hop_inodes)
    #print("eligible_inodes:",eligible_inodes)
    
    eligible_inodes.extend(extra_inodes[:cut_off_point+1])
    
    #print("eligible_inodes:",eligible_inodes)
    
    return eligible_inodes

    pass

    

#def get_vlb_paths_count__pathlen_based_control(vlb_pathlist, sd_pairs, *, inodegenerator, mode_param, N, p, a, twoHopNeighborList = None, sd_pair_vs_4hop_inodes = None, sd_pair_vs_3hop_inodes = None):
def get_vlb_paths_count__pathlen_based_control(**vlb_paths_count_kwargs):

    '''
    pathlen_based_control => a variable for each path length
    
            So for an SD pair, if there are 2 paths of len 6, and 3 paths of len 5,
                there will be two variables. One for len 5 and one for len 6.
    '''
    
    
    '''
    vlb_pathlist: 
        Paths going to be used to calculate vlb-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
        
        *Note, that this is still listing min paths between an SD pair.*
        
        Listing all VLB paths will be too memory-consuming.
        However, a separate pathlist is still necessary because in some cases, we may
        want to use separate min paths lists for min and vlb path load calculation.
        (For example, Df_min paths for min, and djkstra-min for vlb for ugal_restricted.)
    
    inodegenerator: a function that will be used to generate inodes for vlb paths.
            This function can be swapped to support different modes.
                    
    sd_pairs:   traffic pattern under consideration. A list of tuples.
                List of <src_PE, dest_PE>.
    
    N: total routers in the system.
    
    returns a dict of format (src,dst) vs dict2.
            dict2 has the format of {path_len: path_count } 
    '''
    
    #get keyword arguments
    N = vlb_paths_count_kwargs.get("N")
    a = vlb_paths_count_kwargs.get("a")    
    p = vlb_paths_count_kwargs.get("p")
    mode_param = vlb_paths_count_kwargs.get("mode_param")
    
    vlb_pathlist = vlb_paths_count_kwargs.get("vlb_pathlist")
    sd_pairs = vlb_paths_count_kwargs.get("sd_pairs")
    inode_generator = vlb_paths_count_kwargs.get("inode_generator")
    twoHopNeighborList = vlb_paths_count_kwargs.get("twoHopNeighborList")
    sd_pair_vs_4hop_inodes = vlb_paths_count_kwargs.get("sd_pair_vs_4hop_inodes")
    sd_pair_vs_3hop_inodes = vlb_paths_count_kwargs.get("sd_pair_vs_3hop_inodes")
    
    inodegenerator_kwargs = {"N" : N, 
                             "a" : a, 
                             "twoHopNeighborList" : twoHopNeighborList, 
                             "sd_pair_vs_4hop_inodes" : sd_pair_vs_4hop_inodes, 
                             "sd_pair_vs_3hop_inodes" : sd_pair_vs_3hop_inodes, 
                             "mode_param" : mode_param
                             }
    
    vlb_paths_count = {}
    
    for src,dst in sd_pairs:
        
        #test with (0, 50 ) and (22,21)
        if src == -1 and dst == -1:
            flag = True
        else:
            flag = False
        
        if src == dst:
            continue
        
        len_vs_count = {}
        
        src_router = src // p
        dst_router = dst // p
        
        inodes = inode_generator(src_router, dst_router, **inodegenerator_kwargs)
        
        if (flag):
            print(src, dst, inodes)
        
        for imdt in inodes:
            src_imdt_paths = vlb_pathlist[src_router][imdt]
            imdt_dst_paths = vlb_pathlist[imdt][dst_router]
            
            src_imdt_lens = [(len(x) - 1 + 1) for x in src_imdt_paths]
            imdt_dst_lens = [(len(x) - 1 + 1) for x in imdt_dst_paths]
                # - 1 because hop count = node count - 1
                # + 1 to add the src or dst PE as well.
            
            if (flag):
                print(src_imdt_paths)
                print(src_imdt_lens)
                print(imdt_dst_paths)
                print(imdt_dst_lens)
            
            for x in src_imdt_lens:
                for y in imdt_dst_lens:
                    dict_insert(x+y, len_vs_count)
            
        vlb_paths_count[(src,dst)] = sorted(len_vs_count.items())
        if (flag):
            print(vlb_paths_count[(src,dst)])
        
    return vlb_paths_count
    
    
    pass



#def get_vlb_paths_count__all_random_control(vlb_pathlist, sd_pairs, *, inodegenerator, mode_param, N, p, a, twoHopNeighborList = None, sd_pair_vs_4hop_inodes = None):
def get_vlb_paths_count__all_random_control(**vlb_paths_count_kwargs):
    '''
    all_random_control => a variable for each path length
    
            So for an SD pair, if there are 2 paths of len 6, and 3 paths of len 5,
                there will be one variable, and path count will be 5.
    '''
    
    
    '''
    vlb_pathlist: 
        Paths going to be used to calculate vlb-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
        
        *Note, that this is still listing min paths between an SD pair.*
        
        Listing all VLB paths will be too memory-consuming.
        However, a separate pathlist is still necessary because in some cases, we may
        want to use separate min paths lists for min and vlb path load calculation.
        (For example, Df_min paths for min, and djkstra-min for vlb for ugal_restricted.)
    
    inodegenerator: a function that will be used to generate inodes for vlb paths.
            This function can be swapped to support different modes.
                    
    sd_pairs:   traffic pattern under consideration. A list of tuples.
                List of <src_PE, dest_PE>.
    
    N: total routers in the system.
    
    returns a dict of format (src,dst) vs dict2.
            dict2 has the format of {path_len: path_count } 
    '''
    
    #get keyword arguments
    N = vlb_paths_count_kwargs.get("N")
    a = vlb_paths_count_kwargs.get("a")    
    p = vlb_paths_count_kwargs.get("p")
    mode_param = vlb_paths_count_kwargs.get("mode_param")
    
    vlb_pathlist = vlb_paths_count_kwargs.get("vlb_pathlist")
    sd_pairs = vlb_paths_count_kwargs.get("sd_pairs")
    inode_generator = vlb_paths_count_kwargs.get("inode_generator")
    twoHopNeighborList = vlb_paths_count_kwargs.get("twoHopNeighborList")
    sd_pair_vs_4hop_inodes = vlb_paths_count_kwargs.get("sd_pair_vs_4hop_inodes")
    sd_pair_vs_3hop_inodes = vlb_paths_count_kwargs.get("sd_pair_vs_3hop_inodes")
    
    inodegenerator_kwargs = {"N" : N, 
                             "a" : a, 
                             "twoHopNeighborList" : twoHopNeighborList, 
                             "sd_pair_vs_4hop_inodes" : sd_pair_vs_4hop_inodes, 
                             "sd_pair_vs_3hop_inodes" : sd_pair_vs_3hop_inodes, 
                             "mode_param" : mode_param
                             }
    
    vlb_paths_count = {}
    
    for src,dst in sd_pairs:
        
        #test with (0, 50 ) and (22,21)
        if src == -1 and dst == -1:
            flag = True
        else:
            flag = False
        
        if src == dst:
            continue
        
        src_router = src // p
        dst_router = dst // p
        
        inodes = inode_generator(src_router, dst_router, **inodegenerator_kwargs)
        
        if (flag):
            print(src, dst, inodes)
        
        total_paths = 0
        for imdt in inodes:
            src_imdt_paths = vlb_pathlist[src_router][imdt]
            imdt_dst_paths = vlb_pathlist[imdt][dst_router]
            
            total_paths += len(src_imdt_paths) * len(imdt_dst_paths)
            
            if (flag):
                print(src_imdt_paths)
                print(imdt_dst_paths)
                    
        vlb_paths_count[(src,dst)] = [(0, total_paths)]
        
        if (flag):
            print(vlb_paths_count[(src,dst)])
        
    return vlb_paths_count
    
    
    pass

def insert_into_dict_of_dict(dicty, key, tuple):
    '''
    Helper function. Probably for very specific (and limited) purpose.
    
    We have a :
        dict of dict: < (link), dict2>
        dict2 is a dict of <(flow, type, len), count>
    
    First the function will check if the key is present in the dicty.
    If not, it will create an entry dicty[key] = {}
    
    Then it will check if the tuple exists in dicty[key].
    If not, create an entry with value 1.
    Else increment the value.
    '''
    
    if key not in dicty:
        dicty[key] = {}
        
    if tuple in dicty[key]:
        dicty[key][tuple] += 1
    else:
        dicty[key][tuple] = 1
    
    pass


##def track_flows_through_links(sd_pairs, min_pathlist, vlb_pathlist, *, inodegenerator, mode_param, N, a, p, twoHopNeighborList, sd_pair_vs_4hop_inodes, sd_pair_vs_3hop_inodes, min_path_pathlen_based_condtrol, vlb_path_pathlen_based_control):

def track_flows_through_links(min_path_pathlen_based_control, vlb_path_pathlen_based_control, **kwargs):    

    '''
    min_pathlist:
        Paths going to be used to calculate min-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
        
    vlb_pathlist: 
        Paths going to be used to calculate vlb-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
        
        Note, that this is still listing min paths between an SD pair.
        Listing all VLB paths will be too memory-consuming.
        However, a separate pathlist is still necessary because in some cases, we may
        want to use separate min paths lists for min and vlb path load calculation.
        (For example, Df_min paths for min, and djkstra-min for vlb for ugal_restricted.)
    
    inode_generator: a function that will be used to generate inodes for vlb paths.
            This function can be swapped to support different modes.
                    
    sd_pairs:   traffic pattern under consideration. A list of tuples.
                List of <src_PE, dest_PE>.
                
    min_path_pathlen_based_condtrol: boolean. If false, then min pathlen will be stored as 0. 
    
    vlb_path_pathlen_based_control: boolean. If false, then vlb pathlen will be stored as 0.
    
    Returns:
        a dict of dict: < (link), dict2>
        dict2 is a dict of <(flow, type, len), count>
       
    
    '''
    min_pathlist = kwargs.get("min_pathlist")
    vlb_pathlist = kwargs.get("vlb_pathlist")
    inode_generator = kwargs.get("inode_generator")
    sd_pairs = kwargs.get("sd_pairs")
    N = kwargs.get("N")
    a = kwargs.get("a")
    p = kwargs.get("p")
    mode_param = kwargs.get("mode_param")
    twoHopNeighborList = kwargs.get("twoHopNeighborList")
    sd_pair_vs_4hop_inodes = kwargs.get("sd_pair_vs_4hop_inodes")
    sd_pair_vs_3hop_inodes = kwargs.get("sd_pair_vs_3hop_inodes")
   
    link_vs_flow_tracker = {}
    
    
    inodegenerator_kwargs = {"N" : N, 
                             "a" : a, 
                             "twoHopNeighborList" : twoHopNeighborList, 
                             "sd_pair_vs_4hop_inodes" : sd_pair_vs_4hop_inodes, 
                             "sd_pair_vs_3hop_inodes" : sd_pair_vs_3hop_inodes, 
                             "mode_param" : mode_param
                             }
    
    
    for flow_id, (src, dst) in enumerate(sd_pairs):
        
        #just for printing a particular flow details. Set to -1 to effectively disable it. 
        if src == -1 and dst == -1:
            flag = True
        else:
            flag = False
        
        if src == dst:
            continue
        
        src_router = src // p
        dst_router = dst // p
        
        if src_router == dst_router:
            continue
        
        src_group = src_router // a
        dst_group = dst_router // a
        
        #First, update link usage for min paths
        #For min paths, for each hop of each path, save (flow id, "m", pathlen) triplet.
        
        if flag:
            print("src,dst,src_router, dst_router, src_group, dst_group:", src,dst,src_router, dst_router, src_group, dst_group)
        
        min_paths = min_pathlist[src_router][dst_router]
        
        if flag:
            print("min_paths:",min_paths)
        
        for path in min_paths:
            pathlen = len(path) - 1
                    # - 1 because hop count = node count - 1
                    # excluding src and dst PEs
            
            if (flag):
                print("path, pathlen:",path, pathlen)
                
            for ii in range(pathlen):
                hop = (path[ii], path[ii+1])
                
                if min_path_pathlen_based_control == True:
                    insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "m", pathlen + 2))
                                                    # +2 to accomodate the src and dst PEs
                    if flag:
                        print("inserting ", hop, "m", pathlen + 2)        
                else:
                    insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "m", 0))    
                    if flag:
                        print("inserting ", hop, "m", 0)
            
        #Second, update link usage for vlb paths
        #Update: We dont have to explicitly consider ingroup or outgrup pairs. 
        #inode_generator function will take care of that. 
        #Also, we'll use the vlb_pathlist at this stage, which may or may not be
        #equal to the min_pathlist.

        inodes = inode_generator(src_router, dst_router, **inodegenerator_kwargs)
        if flag:
            print("inodes:", inodes)
        
        for imdt in inodes:
            src_imdt_paths = vlb_pathlist[src_router][imdt]
            imdt_dst_paths = vlb_pathlist[imdt][dst_router]
            
            if flag:
                print("src, imdt, dst routers:", src_router, imdt, dst_router)
            
            #get path lens in two arrays
            src_imdt_pathlens = [len(path)-1 + 1 for path in src_imdt_paths]
            imdt_dst_pathlens = [len(path)-1 + 1 for path in imdt_dst_paths]
                                    #-1 because hop count = node count -1
                                    #+1 to include the src or dst PE
            
            if (flag):
                print("src_imdt_paths:",src_imdt_paths)
                print("src_imdt_pathlens:",src_imdt_pathlens)
                print("imdt_dst_paths:",imdt_dst_paths)
                print("imdt_dst_pathlens:",imdt_dst_pathlens)
            
            #update the <src,imdt> hops.
                #For each hop, we need to save the length of the path it is part to.
                #That's why we need a list of all the <imdt,dst> path lengths.
                
            for path_idx,path in enumerate(src_imdt_paths):
                src_imdt_len = src_imdt_pathlens[path_idx]
                if flag:
                    print("path, pathlen", path, src_imdt_len)
                for ii in range(len(path) - 1):
                    hop = (path[ii], path[ii+1])
                    for imdt_dst_len in imdt_dst_pathlens:
                        if vlb_path_pathlen_based_control == True:
                            insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "v", src_imdt_len + imdt_dst_len))
                            if flag:
                                print("inserting ", hop, flow_id, src_imdt_len + imdt_dst_len)
                        else:
                            insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "v", 0))
                            if flag:
                                print("inserting ", hop, flow_id, 0)
                                
            for path_idx,path in enumerate(imdt_dst_paths):
                imdt_dst_len = imdt_dst_pathlens[path_idx]
                if flag:
                    print("path, pathlen", path, imdt_dst_len)
                for ii in range(len(path) - 1):
                    hop = (path[ii], path[ii+1])
                    for src_imdt_len in src_imdt_pathlens:
                        if vlb_path_pathlen_based_control == True:
                            insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "v", src_imdt_len + imdt_dst_len))
                            if flag:
                                print("inserting ", hop, flow_id, src_imdt_len + imdt_dst_len)
                        else:
                            insert_into_dict_of_dict(link_vs_flow_tracker, hop, (flow_id, "v", 0))
                            if flag:
                                print("inserting ", hop, flow_id, 0)
                        
                            
    return link_vs_flow_tracker

    pass



def create_link_rules(link_vs_flow_tracker, verbose = False):
#def create_link_rules(sd_pairs, min_pathlist, vlb_pathlist, *, inodegenerator, N, a, p, twoHopNeighborList = None, test = False):
    
    '''
    For each link, there will be a rule something like this:
        sw_22_1: 1m_54_5
                    + 1v_54_6 + 8v_54_7 + 7v_54_8 + 12v_54_9
                    <= 1
    
    Here: sw_22_1: port 1 of switch 22. We can just name it as link_src_dst. Doesn't matter.
    
        1m_54_5 : 1 min path, of flow id 54, of pathlen 5. m_54_5 is basically a flow variable that we want to optimize.
        
        <= 1: Here 1 is the total capacity of the link.
    
    
    sd_pairs = a list of <src_PE, dst_PE>
    
    
    '''
    
    #We need to keep track of every flow that goes through a link.
    #So having a separate function to do just that might be helpful.
    
    #link_vs_flow_tracker = track_flows_through_links(sd_pairs, min_pathlist, vlb_pathlist, inodegenerator, N, a, p, twoHopNeighborList)
    
    rules = ["" for x in range(len(link_vs_flow_tracker))]
    
    #sort link_vs_flow_tracker based on its key
    sorted_link_vs_flow_tracker = sorted(link_vs_flow_tracker.items())
    
    #for index, (link, dicty) in enumerate(link_vs_flow_tracker.items()):
    for index, (link, dicty) in enumerate(sorted_link_vs_flow_tracker):
         
#        if test:
#            print(link, dicty)
#        
        temp_strings = ["" for x in range(len(dicty))] 
        
        temp = "link{}_{}: ".format(link[0],link[1])
        header = temp
        
        for tup_idx, ((flow_id, pathtype, pathlen), freq) in enumerate(dicty.items()):
            temp = "{}{}_{}_{}".format(freq, pathtype, flow_id, pathlen)
            temp_strings[tup_idx] = temp  
            
        trailer = " <= 1"
        
#        if test:
#            print(temp_strings)
#        
        rules[index] = header + " + ".join(temp_strings) + trailer
        
        if verbose:
            print(rules[index])
            print()
    
    #    for rule in rules:
    #        print(rule)
    #    
    return rules
    
    pass


def create_flow_rules_old(sd_pairs, min_paths_count, vlb_paths_count):
    '''
    For each flow, the rule will look like:
        c0: x - 1m_0_6- 8v_0_7- 14v_0_8- 6v_0_9 <= 0
        
        Here, 1m_0_6 => min path, flow 0, pathlen 6, number of such path 1
              6v_0_9 => vlb path, flow 0, pathlen 9, number of such path 6
              
              x is the optimization objective.
    
    sd_pairs = a list of <src_PE, dst_PE>
    
    '''
    
    rules = ["" for x in range(len(sd_pairs))]
    
    for flow_counter, (src,dst) in enumerate(sd_pairs):
        if src == dst:
            continue
        
        #print("min paths:", min_paths_count[(src,dst)])
        #print("vlb paths:",vlb_paths_count[(src,dst)])
        
        temp_strings = []
        
        temp = "flow{}: x ".format(flow_counter)
        temp_strings.append(temp)
        
        for length, count in min_paths_count[(src,dst)]:
            temp = "- {}m_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        for length, count in vlb_paths_count[(src,dst)]:
            temp = "- {}v_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        temp = "<= 0"
        temp_strings.append(temp)
        
        rules[flow_counter] = " ".join(temp_strings)
    
    return rules
    
    pass

def create_flow_rules(sd_pairs, min_paths_count, vlb_paths_count):
    '''
    For each flow, the rule will look like:
        c0: x - 1m_0_6- 8v_0_7- 14v_0_8- 6v_0_9 <= 0
        
        Here, 1m_0_6 => min path, flow 0, pathlen 6, number of such path 1
              6v_0_9 => vlb path, flow 0, pathlen 9, number of such path 6
              
              x is the optimization objective.
    
    sd_pairs = a list of <src_PE, dst_PE>
    
    '''
    
    rules = ["" for x in range(len(sd_pairs))]
    constraints = []
    
    constraint_count = 0
    
    for flow_counter, (src,dst) in enumerate(sd_pairs):
        if src == dst:
            continue
        
        #print("min paths:", min_paths_count[(src,dst)])
        #print("vlb paths:",vlb_paths_count[(src,dst)])
        
        temp_strings = []
        
        temp = "flow{}: x ".format(flow_counter)
        temp_strings.append(temp)
        
        for length, count in min_paths_count[(src,dst)]:
            temp = "- {}m_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        for length, count in vlb_paths_count[(src,dst)]:
            temp = "- {}v_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        temp = "<= 0"
        temp_strings.append(temp)
        
        rules[flow_counter] = " ".join(temp_strings)
    
        #This is where you add the condition that raters for shorter paths will
        #always be larger than the rates over longer paths.
        
        #don't do anything with min paths
        
        #work with vlb paths only
        
        for idx in range(len(vlb_paths_count[(src,dst)]) -1):
            first_length = vlb_paths_count[(src,dst)][idx][0]
            second_length = vlb_paths_count[(src,dst)][idx+1][0]
            
            constraint_rule = "constraint{}: v_{}_{} - v_{}_{} >= 0".format(constraint_count, flow_counter, first_length, flow_counter, second_length)
            
            constraints.append(constraint_rule)
            constraint_count += 1
            
    return rules, constraints
    
    pass

#TODO: What happens when src == dst? 
#Right now we are just skipping them, but for src or dst rules they might have consequences.
    
def create_source_rules(sd_pairs, min_paths_count, vlb_paths_count, p):
    '''
    For each source, make rules like:
        s_0: 1m_0_6 + 8v_0_7 + 14v_0_8 + 6v_0_9 <= 1
    
    sd_pairs = list of <src_PE, dst_PE> pairs
    '''
    
    rules_dicty = {}
    
    for flow_counter, (src,dst) in enumerate(sd_pairs):
        if src == dst:
            continue
        
        temp_strings = []
        
        for length, count in min_paths_count[(src,dst)]:
            temp = "{}m_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        for length, count in vlb_paths_count[(src,dst)]:
            temp = "{}v_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        if src not in rules_dicty:
            rules_dicty[src] = temp_strings
        else:
            rules_dicty[src].extend(temp_strings)

    #    for k,v in rules_dicty.items():
    #        print(k,v)

    rules = ["" for x in range(len(rules_dicty))]
    
    for idx,(src,strings) in enumerate(sorted(rules_dicty.items())):
        temp_str = "s_{}: ".format(src) + " + ".join(strings) + " <= 1" 
        rules[idx] = temp_str
        
#    for rule in rules:
#        print(rule)
        
    return rules

    pass


def create_destination_rules(sd_pairs, min_paths_count, vlb_paths_count, p):
    '''
    For each destination, make rules like:
        d_0: 1m_16_4 + 1v_16_5 + 3v_16_7 + 12v_16_8 + 12v_16_9 <= 1
    
    sd_pairs = list of <src_PE, dst_PE> pairs
    
    '''
    rules_dicty = {}
    
    for flow_counter, (src,dst) in enumerate(sd_pairs):
        if src == dst:
            continue
        
        temp_strings = []
        
        for length, count in min_paths_count[(src,dst)]:
            temp = "{}m_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        for length, count in vlb_paths_count[(src,dst)]:
            temp = "{}v_{}_{}".format(count, flow_counter, length)
            temp_strings.append(temp)
        
        if dst not in rules_dicty:
            rules_dicty[dst] = temp_strings
        else:
            rules_dicty[dst].extend(temp_strings)

    #    for k,v in rules_dicty.items():
    #        print(k,v)

    rules = ["" for x in range(len(rules_dicty))]
    
    for idx,(dst,strings) in enumerate(sorted(rules_dicty.items())):
        temp_str = "d_{}: ".format(dst) + " + ".join(strings) + " <= 1" 
        rules[idx] = temp_str
        
#    for rule in rules:
#        print(rule)
#        
    return rules

    pass
    

def _select_inode_parameters_for_mcf(**kwargs):
    G = kwargs.get("G")
    a = kwargs.get("a")
    g = kwargs.get("g")
    sd_pairs = kwargs.get("sd_pairs")
    mode = kwargs.get("mode")
    
    #values not likely to change
    p = a//2    #parameterize it later
    
    #default values for arguments which may be changed according to appropriate modes
    twoHopNeighbors = None
    sd_pair_vs_3hop_inodes = None
    sd_pair_vs_4hop_inodes = None
    inode_generator = inode_generator_empty    
    
    #test with (0, 50 ) and (22,21)
    
    if mode == "5hop_paths_src_only":
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
        
        twoHopNeighbors =  DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        inode_generator = inode_generator_5hop_src_only
        
    elif mode == "5hop_paths_src_and_dst":
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
        
        twoHopNeighbors =  DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        inode_generator = inode_generator_5hop_src_and_dst
        
    elif mode == "4hop_paths_only":
        _graph_adj_list = DP.generate_graph_adjacency_list(G)
        _group_pair_vs_nodes = DP.generate_list_of_nodes_connected_to_group_pairs(_graph_adj_list, a)
        
        sd_pair_vs_4hop_inodes = DP.inodes_for_4hop_paths_for_all_SD_pairs(sd_pairs, G, _graph_adj_list, _group_pair_vs_nodes, edge_weight = "w", a = a, g = g, p = p)
        inode_generator = inode_generator_for_4hop_paths
        
    elif mode == "3hop_paths_only":
        _graph_adj_list = DP.generate_graph_adjacency_list(G)
        #group_pair_vs_nodes = DP.generate_list_of_nodes_connected_to_group_pairs(graph_adj_list, a)
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
        
        twoHopNeighbors = DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        sd_pair_vs_3hop_inodes = DP.inodes_for_3hop_paths_for_all_SD_pairs(sd_pairs, G, _graph_adj_list, twoHopNeighbors, edge_weight = "w", a = a, g = g, p = p)
        inode_generator = inode_generator_for_3hop_paths
        
        
    elif mode == "3hop_and_4hop":
        _graph_adj_list = DP.generate_graph_adjacency_list(G)
        _group_pair_vs_nodes = DP.generate_list_of_nodes_connected_to_group_pairs(_graph_adj_list, a)
        
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
        
        twoHopNeighbors =  DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        sd_pair_vs_4hop_inodes = DP.inodes_for_4hop_paths_for_all_SD_pairs(sd_pairs, G, _graph_adj_list, _group_pair_vs_nodes, edge_weight = "w", a = a, g = g, p = p)
        sd_pair_vs_3hop_inodes = DP.inodes_for_3hop_paths_for_all_SD_pairs(sd_pairs, G, _graph_adj_list, twoHopNeighbors, edge_weight = "w", a = a, g = g, p = p)       
        inode_generator = inode_generator_3hop_and_4hop
    
    
    elif mode == "4hop_and_5hop":
        _graph_adj_list = DP.generate_graph_adjacency_list(G)
        _group_pair_vs_nodes = DP.generate_list_of_nodes_connected_to_group_pairs(_graph_adj_list, a)
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
        
        twoHopNeighbors =  DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        sd_pair_vs_4hop_inodes = DP.inodes_for_4hop_paths_for_all_SD_pairs(sd_pairs, G, _graph_adj_list, _group_pair_vs_nodes, edge_weight = "w", a = a, g = g, p = p)
        inode_generator = inode_generator_4hop_and_5hop
    
    
    elif mode == "5hop_and_6hop":
        _group_pair_vs_global_links = DP.get_list_of_links_between_each_group_pair(G, edge_weight = "w", a = a, g = g )
          
        twoHopNeighbors =  DP.generate_list_of_2hop_neighbors(G, _group_pair_vs_global_links, edge_weight = "w", a = a, g = g)
        inode_generator = inode_generator_5hop_and_6hop
                
    
    elif mode == "regular":
        inode_generator = inode_generator_vanilla
    
    elif mode == "minimal_only":
        inode_generator = inode_generator_empty    
    
    elif mode == "vlb_only":
        inode_generator = inode_generator_vanilla  
    
    else:
        print("unsupported mode: ", mode)
        sys.exit(-1)
        
    return twoHopNeighbors, sd_pair_vs_3hop_inodes, sd_pair_vs_4hop_inodes, inode_generator
    

def _perform_mcf_related_path_and_link_counts(min_paths_count_kwargs, vlb_paths_count_kwargs, link_vs_flow_tracker_kwargs, model):
    '''
    model 3: pathlen_based_min, pathlen_based_vlb contol
    model 4: pathlen_based_min, all_random_vlb contol
    model 5: all_random_min, all_random_vlb contol
    '''
    if model == 3:
        #model 3: pathlen_based_min, pathlen_based_vlb contol
        min_paths_count = get_min_paths_count__pathlen_based_control(**min_paths_count_kwargs)
        
        vlb_paths_count = get_vlb_paths_count__pathlen_based_control(**vlb_paths_count_kwargs)
        
        link_vs_flow_tracker = track_flows_through_links(min_path_pathlen_based_control = True, vlb_path_pathlen_based_control = True, **link_vs_flow_tracker_kwargs)


    elif model == 4:
        #model 4: pathlen_based_min, all_random_vlb contol
        min_paths_count = get_min_paths_count__pathlen_based_control(**min_paths_count_kwargs)

        vlb_paths_count = get_vlb_paths_count__all_random_control(**vlb_paths_count_kwargs)
        
        link_vs_flow_tracker = track_flows_through_links(min_path_pathlen_based_control = True, vlb_path_pathlen_based_control = False, **link_vs_flow_tracker_kwargs)

    
    elif model == 5:
        #model 5: all_random_min, all_random_vlb contol
        min_paths_count = get_min_paths_count__pathlen_based_control(**min_paths_count_kwargs)

        vlb_paths_count = get_vlb_paths_count__all_random_control(**vlb_paths_count_kwargs)
        
        link_vs_flow_tracker = track_flows_through_links(min_path_pathlen_based_control = False, vlb_path_pathlen_based_control = False, **link_vs_flow_tracker_kwargs)

        
    else:
        print("unsupported model: ", model)
        sys.exit(-1)
        
    return min_paths_count, vlb_paths_count, link_vs_flow_tracker


def _write_mcf_rules_to_file(filename, flow_rules, constraint_rules, src_rules, dst_rules, link_rules):
    fp = open(filename, "w")
    opening_text = "Maximize\nobj: x\nSubject To\n"
    
    fp.write(opening_text)
    
    for rule in flow_rules:
        if len(rule) > 0:
            fp.write(rule + "\n")
    
    for rule in constraint_rules:
        if len(rule) > 0:
            fp.write(rule + "\n")
    
    for rule in src_rules:
        if len(rule) > 0:
            fp.write(rule + "\n")
    
    for rule in dst_rules:
        if len(rule) > 0:
            fp.write(rule + "\n")
    
    for rule in link_rules:
        if len(rule) > 0:
            fp.write(rule + "\n")
    
    fp.close()
    


#def create_mcf_rules(*,G, a, g, model, minpathlist, vlbpathlist, sd_pairs, filename, mode, mode_param):
def create_mcf_rules(**kwargs):

    '''
    minpathlist: 
        Paths going to be used to calculate min-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
    
    vlbpathlist: 
        Paths going to be used to calculate vlb-path loads.
        A 3D array where paths are stored as [src][dst][min paths between src and dst].
        Each path is a list of tuples.
        Note, that this is still listing min paths between an SD pair.
        Listing all VLB paths will be too memory-consuming.
        However, a separate pathlist is still necessary because in some cases, we may
        want to use separate min paths lists for min and vlb path load calculation.
        (For example, Df_min paths for min, and djkstra-min for vlb for ugal_restricted.)
    
    filename: the file where lp formulation is to be written
    
    sd_pairs:   traffic pattern. A list of <src,dst> tuples.
                Used to be <src_router, dst_router>. Now changing to <src_pe, dst_pe> pairs.
    
    mode: can be "minimal_only", "vlb_only", "4hop_paths_only", "5hop_paths_src_only", "5hop_paths_src_and_dst", "regular", "5hop_and_6hop"
    
    mode_param: only relevant for mode "5hop_and_6hop". Means the % of 6hops inodes to include. Will be 
                ignored by every other mode.
                
    model: mcf model. Current supported values: 3, 4, 5
            model 3: pathlen_based_min, pathlen_based_vlb contol
            model 4: pathlen_based_min, all_random_vlb contol
            model 3: all_random_min, all_random_vlb contol
            
    
    '''
    #Step 0: get the keyworded parameters, set to None if not found
    G = kwargs.get("G")
    a = kwargs.get("a")
    g = kwargs.get("g")
    model = kwargs.get("model")
    minpathlist = kwargs.get("minpathlist")
    vlbpathlist = kwargs.get("vlbpathlist")
    sd_pairs = kwargs.get("sd_pairs")
    filename = kwargs.get("filename")
    mode = kwargs.get("mode")
    mode_param = kwargs.get("mode_param")
    
    #values not likely to change
    p = a//2    #parameterize it later
    N = len(G)
    
    #step 1: get the necessary helper functions and data structures prepared
    twoHopNeighbors, sd_pair_vs_3hop_inodes, sd_pair_vs_4hop_inodes, inode_generator = _select_inode_parameters_for_mcf(**kwargs)
    
    
    #step 2: get the necessary arguments ready
    
    #bad design. This needs to be delegated somewhere else.
    if mode == "vlb_only":
        minpathlist = [ [ [] for x in range(N) ] for x in range(N) ]
            # This is okay because we are changing the reference inside the function. 
            # So this will not change the pathlist in the parent function.
            # Also, each SD pair has a blank minpathlist, means only vlb_paths will be considered in the calculations.
        
    
    #now create the argument-lists to pass to the functions.
    min_paths_count_kwargs = {"min_pathlist" : minpathlist, 
                              "sd_pairs" : sd_pairs, 
                              "N" : N, 
                              "p" : p}
    
    vlb_paths_count_kwargs = {
                              "vlb_pathlist" : vlbpathlist, 
                              "sd_pairs": sd_pairs, 
                              "N" : N, 
                              "p" : p, 
                              "a" : a, 
                              "mode_param" : mode_param, 
                              #these ones vary depending on mode
                              "inode_generator" : inode_generator, 
                              "twoHopNeighborList" : twoHopNeighbors, 
                              "sd_pair_vs_4hop_inodes" : sd_pair_vs_4hop_inodes, 
                              "sd_pair_vs_3hop_inodes" : sd_pair_vs_3hop_inodes}
    
    link_vs_flow_tracker_kwargs = {"sd_pairs" : sd_pairs, 
                                   "min_pathlist" : minpathlist, 
                                   "vlb_pathlist" : vlbpathlist, 
                                   "inode_generator" : inode_generator, 
                                   "mode_param" : mode_param, 
                                   "N" : N, 
                                   "a" : a, 
                                   "p" : p, 
                                   "twoHopNeighborList" : twoHopNeighbors, 
                                   "sd_pair_vs_4hop_inodes" : sd_pair_vs_4hop_inodes, 
                                   "sd_pair_vs_3hop_inodes" : sd_pair_vs_3hop_inodes
                                   }    
    
    #step 3: get the mcf related path and link load pre-calculations done
    
    min_paths_count, vlb_paths_count, link_vs_flow_tracker = _perform_mcf_related_path_and_link_counts(min_paths_count_kwargs, vlb_paths_count_kwargs, link_vs_flow_tracker_kwargs, model)


    #step 4: generate mcf rules

    flow_rules, constraint_rules = create_flow_rules(sd_pairs, min_paths_count, vlb_paths_count)    
        
    src_rules = create_source_rules(sd_pairs, min_paths_count, vlb_paths_count, p)    
    
    dst_rules = create_destination_rules(sd_pairs, min_paths_count, vlb_paths_count, p)    

    link_rules = create_link_rules(link_vs_flow_tracker)

    
    #step 5: write rules to file
    _write_mcf_rules_to_file(filename, flow_rules, constraint_rules, src_rules, dst_rules, link_rules)
    
    
    pass


if __name__ == "__main__":
    print("Hello world")
    
    print("The world never says hello back.")