#include "cop_lib/cop.h"

using namespace coplibrary;

COP::COP(std::shared_ptr <Graph> graph, size_t const start_vertex, double const budget, MatDbl const &costs) : 
    graph_{graph}, 
    start_vertex_{start_vertex}, 
    budget_{budget}, 
    costs_{costs} 
{
    n_ = graph_->num_vertices;
    Solve();
}


void COP::Initialize() {
    cop_vertices_.push_back(start_vertex_);
    greedy_seq_.push_back(start_vertex_);
    for(size_t v_id = 0; v_id < n_; ++v_id) { // v_id for vertex ID
        auto &vertex = graph_->vertices[v_id]; // Reference variable for convenience

        /* Create a new set of vertices called potential vertices (pot_vertices_).
            * These contain vertices with a positive reward and are reachable from the initial location of the robot within the budget constraint */
        if(not(vertex.reward >= 0) or start_vertex_ == v_id) {
            continue;
        }

        vertex.cost = costs_[start_vertex_][v_id] + costs_[v_id][start_vertex_];
        assert(vertex.cost >= 0);
        if(vertex.cost > budget_) {
            continue;
        }

        /* Compute neighbors and co-neighbors
            * Neighbors N(v_id) of a vertex v_id is the set of vertices
            * such that visiting a vertex in N(v_id) gives some reward
            * associated with vertex v_id
            *
            * Co-neighbors of a vertex v_id is the set of vertices
            * such that visiting the vertex v_id gives some reward
            * associated with the vertices in N(v_id) */
        std::list <VW> neighbors, co_neighbors;
        for(auto const &e:graph_->edge_list) {
            if(e.v == v_id and e.w > 0) {
                neighbors.push_back(VW{e.u, e.w});
            }
            if(e.u == v_id and e.w > 0 and graph_->vertices[e.v].reward > 0) {
                co_neighbors.push_back(VW{e.v, e.w});
            }
        }
        vertex.neighbors = neighbors;
        vertex.co_neighbors = co_neighbors;

        /* Initially, the total utility is the amount of reward we would get by
            * traversing a vertex v_id as though none of the other vertices are traversed
            * This is equal to reward of v_id + reward from co-neighbors */
        vertex.utility = vertex.reward;
        for (auto const &vw:co_neighbors) {
            vertex.utility += vw.w * graph_->vertices[vw.v].reward;
        }
        assert(vertex.utility >= 0);
        if(vertex.utility > 0)  {
            if(vertex.cost > 0)  {
                vertex.uc_ratio = vertex.utility / vertex.cost;
                pot_vertices_.push_back(v_id);
            } else { // cost == 0, i.e., distance to v_id is 0; can visit immediately
                cop_vertices_.push_back(v_id);
            }
        }
    }
    tsp_cost_matrix_.resize(cop_vertices_.size() + 1, VecDbl(cop_vertices_.size() + 1, 0));
    /* tsp_cost_matrix_.reserve(cop_vertices_.size()); */
    /* for(auto const &u:cop_vertices_) { */
    /* 	VecDbl costs_row; */
    /* 	costs_row.reserve(cop_vertices_.size()); */
    /* 	for(auto const &v:cop_vertices_) { */
    /* 		costs_row.push_back(costs_[u][v]); */
    /* 	} */
    /* 	tsp_cost_matrix_.push_back(costs_row); */
    /* } */
}

void COP::Solve() {
    Initialize();
    while(pot_vertices_.size() > 0) {

        /* Extract best vertex to add to route as per the ratio of utility and delta cost */
        auto v_itr = std::max_element(pot_vertices_.begin(), pot_vertices_.end(), [this](size_t const &i, size_t const &j) {return graph_->vertices[i].uc_ratio < graph_->vertices[j].uc_ratio;});

        size_t best_v_id = *v_itr;
        pot_vertices_.erase(v_itr);
        cop_vertices_.push_back(best_v_id);
        std::cout << "best vertex id: " << best_v_id << std::endl;
        auto &best_v = graph_->vertices[best_v_id];
        std::cout << "best v reward:" << best_v.reward << std::endl;
        size_t last_idx = cop_vertices_.size() - 1;
        size_t idx = 0;
        for(auto const v:cop_vertices_) {
            tsp_cost_matrix_[idx][last_idx] = costs_[v][best_v_id];
            tsp_cost_matrix_[last_idx][idx] = costs_[best_v_id][v];
            ++idx;
        }
        greedy_seq_.push_back(best_v_id);

        if(pot_vertices_.size() == 0) {
            break;
        }

        /* Change utility of neighbors */
        for(auto const vw:best_v.neighbors) {
            auto &v_j = graph_->vertices[vw.v];
            v_j.utility = std::max(0., v_j.utility - vw.w * best_v.reward);
        }

        /* Change utility of co-neighbors */
        for(auto const vw:best_v.co_neighbors) {
            auto &v_j = graph_->vertices[vw.v];
            v_j.utility = std::max(0., v_j.utility - vw.w * v_j.reward);
        }

        /* Remove best vertex from neighbors and co-neighbors of all potential vertices */
        for(auto const v:pot_vertices_) {
            graph_->vertices[v].neighbors.remove_if([best_v_id](VW const &vw){ return vw.v == best_v_id; });
            graph_->vertices[v].co_neighbors.remove_if([best_v_id](VW const &vw){ return vw.v == best_v_id; });
        }

        VecDbl row_cost(cop_vertices_.size() + 1, 0);
        for(size_t idx = 0; idx < cop_vertices_.size(); ++idx) {
            tsp_cost_matrix_[idx].push_back(0);
        }
        tsp_cost_matrix_.push_back(row_cost);
        /* Compute the cost of route for adding a potential edge */
        for(auto const v:pot_vertices_) {
            double route_cost;
            ComputeRoute(v, route_cost, tsp_cost_matrix_);
            graph_->vertices[v].cost = route_cost;
        }
        pot_vertices_.remove_if([this](size_t v) { return graph_->vertices[v].cost > budget_; });
    }

    ATSP_DP_BHK tsp_solver(cop_vertices_.size(), tsp_cost_matrix_);
    route_cost_ = tsp_solver.GetCost();
    std::cout << "[cop solver] the route cost is " << route_cost_ << std::endl;
    VecUint tsp_seq;
    tsp_solver.GetPath(tsp_seq);
    for(auto const &v:tsp_seq) {
        path_.push_back(greedy_seq_[v]);
    }
}

std::vector<size_t> COP::GetPath() { return path_; }


void COP::ComputeRoute(size_t const new_vertex, double &route_cost, MatDbl tsp_cost_matrix) {
    if(cop_vertices_.size() == 1) {
        route_cost = costs_[cop_vertices_.front()][new_vertex] + costs_[new_vertex][cop_vertices_.front()];
    }
    size_t last_idx = cop_vertices_.size() - 1;
    size_t idx = 0;
    for(auto const v:cop_vertices_) {
        tsp_cost_matrix[idx][last_idx] = costs_[v][new_vertex];
        tsp_cost_matrix[last_idx][idx] = costs_[new_vertex][v];
        ++idx;
    }
    ATSP_DP_BHK tsp_solver(cop_vertices_.size(), tsp_cost_matrix);
    route_cost = tsp_solver.GetCost();
}