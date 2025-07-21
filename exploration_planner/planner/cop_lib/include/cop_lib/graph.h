#ifndef _COP_GRPAH_
#define _COP_GRPAH_

#include "cop_lib/typedefs.h"
#include <list>

namespace coplibrary {

	struct VW { size_t v; double w; };

	struct Edge {
		size_t u, v;
		double w;
	};

	struct Vertex {
		size_t ID = 0;
		double reward = 0;
		double utility = 0, cost = 0;
		double uc_ratio = 0;
		std::list <VW> neighbors;
		std::list <VW> co_neighbors;
		Vertex(size_t const ID_in, double const r) : ID{ID_in}, reward{r} {  }
	};

	struct Graph {
		size_t num_vertices;
		VecT <Vertex> vertices;
		VecT <Edge> edge_list;
		Graph(size_t const n_in):num_vertices{n_in}{ vertices.reserve(num_vertices); }
	};

}
#endif /* _COP_GRPAH_ */

