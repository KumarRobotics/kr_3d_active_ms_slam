#ifndef _COP_APSP_FLOYDWARSHALL_H_
#define _COP_APSP_FLOYDWARSHALL_H_

#include <vector>
#include <memory>
#include "cop_lib/graph.h"
#include "cop_lib/typedefs.h"

namespace coplibrary {

class APSP_FloydWarshall {
	private:
		std::shared_ptr <Graph const> g_;
		size_t n_ = 0;
		bool is_directed_ = false;

		MatDbl distances_;
		MatUint spath_node_;

		void GetPathAux(size_t const i, size_t const j, VecUint &path) const {
			if (i == j) { return; }

			if (spath_node_[i][j] == j) { path.push_back(j); return; }
			GetPathAux(i, spath_node_[i][j], path);
			GetPathAux(spath_node_[i][j], j, path);
		}

	public:

		APSP_FloydWarshall(std::shared_ptr <Graph const> const g,  bool is_directed=false) : g_{g}, is_directed_{is_directed} {
			n_ = g_->num_vertices;
			distances_.resize(n_, VecDbl(n_, kInf));
			spath_node_.resize(n_, VecUint (n_, kNIL));
			Compute();
		}

		void Compute() {
			for(size_t i = 0; i < n_; ++i) {
				distances_[i][i] = 0;
				spath_node_[i][i] = i;
			}

			for(auto const e:g_->edge_list) {
				distances_[e.u][e.v] = e.w;
				spath_node_[e.u][e.v] = e.v;
				if(is_directed_ == false) {
					distances_[e.v][e.u] = e.w;
					spath_node_[e.v][e.u] = e.u;
				}
			}

			for(size_t k = 0; k < n_; ++k) {
				for(size_t i = 0; i < n_; ++i) {
					for(size_t j = 0; j < n_; ++j) {
						if(distances_[i][k] + distances_[k][j] < distances_[i][j]) {
							distances_[i][j] = distances_[i][k] + distances_[k][j];
							spath_node_[i][j] = k;
						}
					}
				}
			}
		}

		bool GetPath(size_t const i, size_t const j, VecUint &path) const {
			if (spath_node_[i][j] == kNIL) {
				return 1;
			}
			path.push_back(i);
			GetPathAux(i, j, path);
			return 0;
		}

		double GetCost(size_t const i, size_t const j) const {
			return distances_[i][j];
		}

		void GetCostMatrix(MatDbl &cost_matrix) const {
			cost_matrix = distances_;
		}

};

} /* namespace coplibrary */

#endif /* _APSP_FLOYDWARSHALL_H_ */
