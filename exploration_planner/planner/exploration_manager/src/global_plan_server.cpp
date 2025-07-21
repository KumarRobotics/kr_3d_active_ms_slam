#include "exploration_manager/global_plan_server.h"

using namespace ms_planner;

GlobalPlanServer::GlobalPlanServer(ros::NodeHandle &nh_, ros::NodeHandle &pnh_) : 
  nh(nh_),
  pnh(pnh_)
{
  // Topics
  nh.param<std::string>("global_plan_server", global_plan_server_topic_, "/exploration/global_plan_server");

  // Action server
  global_plan_server_ptr_.reset(new GlobalPlanServerType(global_plan_server_topic_, boost::bind(&GlobalPlanServer::receiveGlobalPlanGoalCB, this, _1), false));
  global_plan_server_ptr_->start();
}

GlobalPlanServer::~GlobalPlanServer() {};


void GlobalPlanServer::receiveGlobalPlanGoalCB(const exploration_msgs::PlanCOPGoal::ConstPtr &goal) {
  ROS_INFO("[GlobalPlanServer]: Received global plan goal.");
  
  // extract the information
  int dimension = goal->mat_size;
  // int problem_size = std::min(11, dimension);
  int problem_size = goal->problem_size;
  std::shared_ptr<coplibrary::Graph> env_graph = std::make_shared<coplibrary::Graph>(problem_size);
  for (size_t i = 0; i < problem_size; ++i)
  {
    for (size_t j = 0; j < problem_size; ++j)
    {
      if (i == j)
      {
        continue;
      }
      env_graph->edge_list.push_back(coplibrary::Edge{i, j, goal->cost_mat[i*dimension + j]});
    }
  }
  /* Compute cost matrix using APSP solver */
  coplibrary::APSP_FloydWarshall apsp(env_graph, false);
  coplibrary::MatDbl cost_matrix;
  apsp.GetCostMatrix(cost_matrix);

  // create cop graph
  ROS_INFO("[EXP MANAGER] create cop graph");
  std::cout << "info gain size: " << goal->info_gain.size() << std::endl;
  std::cout << "input mat dimension: " << dimension << std::endl;
  std::cout << "use problem_size: " << problem_size << std::endl;
  std::shared_ptr<coplibrary::Graph> cop_graph = std::make_shared<coplibrary::Graph>(problem_size);
  // Current position as idx 0. Frontier start from idx 1
  cop_graph->vertices.push_back(coplibrary::Vertex(0, 0));
  for (size_t i = 1; i < problem_size; ++i)
  {
    // Add vertices
    cop_graph->vertices.push_back(coplibrary::Vertex(i, goal->info_gain[i]));
  }
  // add edges with positive correlation
  for (size_t i = 0; i < problem_size; ++i)
  {
    for (size_t j = 0; j < problem_size; ++j)
    {
      if (i == j)
      {
        continue;
      }
      cop_graph->edge_list.push_back(coplibrary::Edge{i, j, goal->correlation_mat[i*dimension + j]});
    }
  }

  size_t start_vertex = 0;

  // ROS_INFO("[EXP MANAGER] solve COP, the budget is, %f", budget);
  double budget = goal->budget;
  coplibrary::COP cop(cop_graph, start_vertex, budget, cost_matrix);
  auto path = cop.GetPath();
  ROS_INFO("[EXP MANAGER] COP results");
  // std::for_each(path.begin(), path.end(), [](size_t v) {std::cout << v << " ";});
  for (int i = 0; i < path.size(); i++)
  {
    ROS_INFO_STREAM("[EXP MANAGER] COP results: " << path[i] << " ");
  }

  // Send back the result
  exploration_msgs::PlanCOPResult result;
  result.success = true;
  for (int i = 0; i < path.size(); i++)
  {
    result.sequence.push_back(path[i]);
  }
  // Copy the data from action goal back to the result.
  result.budget = budget;
  result.cost_mat = goal->cost_mat;
  result.correlation_mat = goal->correlation_mat;
  ROS_INFO("[GlobalPlanServer]: Global plan goal succeeded.");
  global_plan_server_ptr_->setSucceeded(result);

}