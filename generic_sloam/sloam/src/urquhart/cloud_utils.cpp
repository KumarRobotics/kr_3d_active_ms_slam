#include <cloud_utils.hpp>

PointVector makePointVector(const std::vector<Cylinder> &obs) {
  PointVector pv;
  for (const auto &tree : obs) {
    vecPtT root{tree.model.root[0], tree.model.root[1], tree.model.root[2]};
    pv.push_back(root);
  }
  return pv;
}

PointVector filterPoints(const PointVector &pv) {
  // remove duplicate trees
  PointVector validTrees;
  for (const auto &a : pv) {
    bool dup = false;
    for (const auto &b : validTrees) {
      if (euclideanDistance(a, b) < 1.0) {
        dup = true;
        break;
      }
    }
    if (!dup) validTrees.push_back(a);
  }
  return validTrees;
}

void makeCylinderPointClouds(const std::vector<idxPair> &matches,
                             const std::vector<Cylinder> &source,
                             const std::vector<Cylinder> &target, const SE3 &tf,
                             const double inlier_thresh, CloudT::Ptr src_pc,
                             CloudT::Ptr tgt_pc) {
  // find all inlier associations
  for (const auto m : matches) {
    // std::cout << "MATCH" << m.first << " x " << m.second << std::endl;
    PointT src_pt;
    PointT tgt_pt;
    Cylinder src_cyl = source[m.first];
    src_cyl.project(tf);
    src_pt.x = src_cyl.model.root[0];
    src_pt.y = src_cyl.model.root[1];
    src_pt.z = src_cyl.model.root[2];

    const auto &tgt_cyl = target[m.second];
    tgt_pt.x = tgt_cyl.model.root[0];
    tgt_pt.y = tgt_cyl.model.root[1];
    tgt_pt.z = tgt_cyl.model.root[2];

    // add corresponding cylinder points to new point clouds
    // std::cout<< "D: " << euclideanDistance(src_pt, tgt_pt) << std::endl;
    if (euclideanDistance(src_pt, tgt_pt) < inlier_thresh) {
      for (auto pt : src_cyl.features) src_pc->points.push_back(pt);

      for (auto pt : tgt_cyl.features) tgt_pc->points.push_back(pt);
    }
  }

  src_pc->height = 1;
  src_pc->width = src_pc->points.size();

  tgt_pc->height = 1;
  tgt_pc->width = tgt_pc->points.size();
}

void makePointClouds(const std::vector<idxPair> &matches,
                     const std::vector<Cylinder> &source,
                     const std::vector<Cylinder> &target, CloudT::Ptr src_pc,
                     CloudT::Ptr tgt_pc) {
  for (const auto &m : matches) {
    PointT src_pt;
    PointT tgt_pt;

    const auto &src_tree = source[m.first];
    src_pt.x = src_tree.model.root[0];
    src_pt.y = src_tree.model.root[1];
    src_pt.z = src_tree.model.root[2];

    const auto &tgt_tree = target[m.second];
    tgt_pt.x = tgt_tree.model.root[0];
    tgt_pt.y = tgt_tree.model.root[1];
    tgt_pt.z = tgt_tree.model.root[2];

    src_pc->points.push_back(src_pt);
    tgt_pc->points.push_back(tgt_pt);
  }

  src_pc->height = 1;
  src_pc->width = src_pc->points.size();

  tgt_pc->height = 1;
  tgt_pc->width = tgt_pc->points.size();
}

void makePointClouds(std::vector<std::pair<vecPtT, vecPtT>> &matches,
                     CloudT::Ptr src_pc, CloudT::Ptr tgt_pc) {
  for (const auto &m : matches) {
    PointT src_pt;
    PointT tgt_pt;

    src_pt.x = m.first[0];
    src_pt.y = m.first[1];
    src_pt.z = m.first[2];

    tgt_pt.x = m.second[0];
    tgt_pt.y = m.second[1];
    tgt_pt.z = m.second[2];

    src_pc->points.push_back(src_pt);
    tgt_pc->points.push_back(tgt_pt);
  }

  src_pc->height = 1;
  src_pc->width = src_pc->points.size();

  tgt_pc->height = 1;
  tgt_pc->width = tgt_pc->points.size();
}