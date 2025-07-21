#ifndef _COP_H_
#define _COP_H_
/* Solves the Correlated Orienteering Problem
 * Inputs:
 *			Cost matrix of size n x n. Needs to be shortest path
 */

#include "cop_lib/typedefs.h"
#include "cop_lib/atsp_held_karp.h"
#include "cop_lib/graph.h"

#include <memory>
#include <algorithm>
#include <functional>
#include <iostream>
#include <cassert>
#include <list>

namespace coplibrary {

    /* Struct for potential vertices that can be added to the path */
    class COP {
        private:
            std::shared_ptr <Graph> graph_; // Assumes a directed graph
            size_t start_vertex_;
            double budget_;
            MatDbl costs_;
            size_t n_ = 0;
            std::list <size_t> pot_vertices_;
            double route_cost_ = 0;
            std::list <size_t> cop_vertices_;
            MatDbl tsp_cost_matrix_; /* matrix of costs for TSP */
            VecUint path_;
            VecUint greedy_seq_;

            void Initialize();

        public:
            COP (std::shared_ptr <Graph> graph, size_t const start_vertex, double const budget, MatDbl const &costs);

            void Solve();

            std::vector<size_t> GetPath();

            void ComputeRoute(size_t const new_vertex, double &route_cost, MatDbl tsp_cost_matrix);
    };

} /* namespace coplibrary */

#endif /* _COP_H_ */
