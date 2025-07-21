#pragma once

#include <cylinder.h>

#include <distance.hpp>

PointVector makePointVector(const std::vector<Cylinder> &obs);

PointVector filterPoints(const PointVector &pv);

void makeCylinderPointClouds(const std::vector<idxPair> &matches,
                             const std::vector<Cylinder> &source,
                             const std::vector<Cylinder> &target, const SE3 &tf,
                             const double inlier_thresh, CloudT::Ptr src_pc,
                             CloudT::Ptr tgt_pc);

void makePointClouds(const std::vector<idxPair> &matches,
                     const std::vector<Cylinder> &source,
                     const std::vector<Cylinder> &target, CloudT::Ptr src_pc,
                     CloudT::Ptr tgt_pc);

void makePointClouds(std::vector<std::pair<vecPtT, vecPtT>> &matches,
                     CloudT::Ptr src_pc, CloudT::Ptr tgt_pc);