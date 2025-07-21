/**
 * The file contains DP algorithm for ATSP given by Bellman-Held-Karp
 *
 * TODO:
 *
 * MetaData:
 * @author Saurav Agarwal
 * @contact SauravAg@upen.edu
 * @contact agr.saurav1@gmail.com
 * Original code from the repository: https://github.com/UNCCharlotte-Robotics/LineCoverage-library
 *
 */

#ifndef _ROUTING_ALGORITHMS_ATSP_DP_BHK_H_
#define _ROUTING_ALGORITHMS_ATSP_DP_BHK_H_

#include <vector>
#include <cmath>
#include "cop_lib/typedefs.h"

namespace coplibrary {


	class ATSP_DP_BHK {

		struct Path {
			size_t v;
			size_t s;
			double c;
			Path() : v{kNIL}, s{kNIL}, c{kInf} {}
		};


		size_t n_;
		const std::vector < std::vector <double> > &d_;
		std::vector < std::vector <Path> > subpath_;
		size_t max_subsets_;
		double cost_;

		void SolveATSP(size_t v, size_t s) {
			size_t min_v = kNIL;
			size_t s_min = kNIL;

			for(size_t i = 0; i < n_; ++i) {

				if(i == v || (s & (1 << i))) {
					continue;
				}

				size_t s_new = (s | (1 << i));

				double path_cost = d_[v][i];
				if(s_new == max_subsets_) {
					path_cost += d_[i][0];
				}
				else if(subpath_[i][s_new].v == kNIL) {
					SolveATSP(i, s_new);
					path_cost += subpath_[i][s_new].c;
				}
				else {
					path_cost += subpath_[i][s_new].c;
				}

				if(path_cost < subpath_[v][s].c) {
					subpath_[v][s].c = path_cost;
					min_v = i;
					s_min = s_new;
				}
			}
			subpath_[v][s].v = min_v;
			subpath_[v][s].s = s_min;
		}

		public:
		ATSP_DP_BHK(const size_t n, const std::vector < std::vector <double> > &d) : n_{n}, d_{d} {
			max_subsets_ = (1 << n) - 1;
			subpath_.resize(n, std::vector <Path>(max_subsets_));
			SolveATSP(0, 1);
			cost_ = subpath_[0][1].c;
		}


		int GetPath(std::vector < size_t > &path) const {
			path.push_back(0);
			double cost = 0;
			size_t v = 0; size_t s = 1;
			for(size_t i = 0; i < n_ - 1; ++i) {
				auto p = subpath_[v][s];
				cost += d_[v][p.v];
				v = p.v;
				s = p.s;
				path.push_back(v);
			}
			path.push_back(0);
			cost += d_[v][0];
			if(path.size() != (n_ + 1))
				return 1;
			if(not(std::abs(cost-cost_) < 1e-10))
				return 1;
			return 0;
		}

		double GetCost() { return cost_; }

	};

} // namespace coplibrary

#endif /* _ROUTING_ALGORITHMS_ATSP_DP_BHK_H_ */
