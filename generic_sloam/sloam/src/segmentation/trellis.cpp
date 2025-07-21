#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <trellis.h>

#include <chrono>

Instance::Instance() { tree_id_ = 0; }

void Instance::findClusters(const CloudT::Ptr pc,
                            pcl::PointCloud<pcl::Label> &euclidean_labels,
                            std::vector<pcl::PointIndices> &label_indices) {
  if (pc->size() == 0) return;
  pcl::EuclideanClusterComparator<PointT, pcl::Label>::Ptr
      euclidean_cluster_comparator(
          new pcl::EuclideanClusterComparator<PointT, pcl::Label>());

  euclidean_cluster_comparator->setInputCloud(pc);
  // TODO: make this a param
  double connected_component_threshold = 1.0;
  euclidean_cluster_comparator->setDistanceThreshold(
      connected_component_threshold, false);

  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
      euclidean_segmentation(euclidean_cluster_comparator);
  euclidean_segmentation.setInputCloud(pc);
  // label_indices is a vector of PointIndices corresponding to each label /
  // component id
  euclidean_segmentation.segment(euclidean_labels, label_indices);

}

TreeVertex Instance::computeTreeVertex(CloudT::Ptr beam, int label) {
  TreeVertex v;
  Slash filteredPoints;
  PointT median;
  Scalar radius;
  bool valid = computeVertexProperties(beam, filteredPoints, median, radius);

  v.treeId = label;
  v.prevVertexSize = 0;
  v.points = filteredPoints;
  v.coords = median;
  // v.isRoot = false;
  v.isValid = valid;
  v.beam = 0;
  v.radius = radius;
  return v;
}

bool Instance::computeVertexProperties(CloudT::Ptr &pc, Slash &filteredPoints,
                                       PointT &median_point, Scalar &radius) {
  // Compute median in each x,y,z
  int num_points = pc->points.size();
  int middle_point = (int)(num_points / 2.0);
  Scalar median_x = 0;
  Scalar median_y = 0;
  Scalar median_z = 0;

  std::sort(pc->points.begin(), pc->points.end(),
            [](const PointT &p1, const PointT &p2) { return p1.x < p2.x; });

  median_x = pc->points[middle_point].x;

  std::sort(pc->points.begin(), pc->points.end(),
            [](const PointT &p1, const PointT &p2) { return p1.y < p2.y; });
  median_y = pc->points[middle_point].y;

  std::sort(pc->points.begin(), pc->points.end(),
            [](const PointT &p1, const PointT &p2) { return p1.z < p2.z; });
  median_z = pc->points[middle_point].z;

  // PointT median_point;
  median_point.x = median_x;
  median_point.y = median_y;
  median_point.z = median_z;

  for (const auto &point : pc->points) {
    if (euclideanDistance(point, median_point) < params_.max_dist_to_centroid) {
      filteredPoints.push_back(point);
    }
  }

  if (filteredPoints.size() > 1) {
    PointT pointA = filteredPoints[0];
    PointT pointB = filteredPoints[filteredPoints.size() - 1];
    radius = euclideanDistance(pointA, pointB);
    return true;
  }
  return false;
}

void Instance::findTrees(const CloudT::Ptr pc,
                         pcl::PointCloud<pcl::Label> &euclidean_labels,
                         std::vector<pcl::PointIndices> &label_indices,
                         std::vector<std::vector<TreeVertex>> &landmarks) {

  for (size_t i = 0; i < label_indices.size(); i++) {
    // top layer: iterate through all labels
    int cluster_min_size = 3;
    int min_rows_for_tree = 2;
    int min_beam_size = 1;
    if (label_indices.at(i).indices.size() > cluster_min_size) {
      std::vector<TreeVertex> tree;
      for (int row_idx = pc->height - 1; row_idx >= 0; --row_idx) {
        // middle layer: iterate all rows, build trellis graph
        CloudT::Ptr beam(new CloudT);

        // step 1: beams (clusters of points at the same row) from i-th cluster
        for (size_t col_idx = 0; col_idx < pc->width; ++col_idx) {
          // bottom layer: iterate all columns, get points from i-th cluster
          if (euclidean_labels.points[row_idx * pc->width + col_idx].label ==
              i) {
            // add points belonging to i-th cluster
            PointT p = pc->at(col_idx, row_idx);
            beam->points.push_back(p);
          }
        }

        // step 2: record current beam inside current tree
        if (beam->points.size() >= min_beam_size) {
          TreeVertex v = computeTreeVertex(beam, i);
          if (v.isValid) {
            tree.push_back(v);
          }
        }
      }

      if (tree.size() > min_rows_for_tree) {
        // if more than min_rows_for_tree rows belong to the same tree, then
        // regard it as a valid tree
        if (tree.size() > 200) {
          tree.resize(200);
        }

        landmarks.push_back(tree);
      }
    }
  }
}

void Instance::computeGraph(const CloudT::Ptr tree_cloud,
                            std::vector<std::vector<TreeVertex>> &landmarks) {
  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> label_indices;
  findClusters(tree_cloud, euclidean_labels, label_indices);
  // label_indices is a vector of PointIndices corresponding to each label /
  // component id

  // euclidean_labels is a PointCloud of labels: each connected component
  // will have a unique id
  findTrees(tree_cloud, euclidean_labels, label_indices, landmarks);
}