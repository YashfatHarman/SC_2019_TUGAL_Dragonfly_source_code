#ifndef _Pair_hash_HPP_
#define _Pair_hash_HPP_

#include <functional>

//this is to facilitate the use of an <int, int> pair as the key of an unordered_map.
//STL does not know how to hash a pair, so using a pair as key throws up an error.
//To eliminate that, you need to provide your own hash function.
//Alternatively, we could use the boost::hash function, but that would require boost
//library to be installed in every machine we run the code. Too much overhead.

struct pair_hash{
    // long int operator() (const std::pair<int, int> &p) const{
    //     long int res = p.first * 100000 + p.second;
    //     return res;
    // }
    std::size_t operator() (const std::pair<int, int> &p) const{
        std::size_t res = std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
        return res;
    }
};

#endif