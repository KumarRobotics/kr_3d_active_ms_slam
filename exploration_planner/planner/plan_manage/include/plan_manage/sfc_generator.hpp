#ifndef _SFC_GENERATOR_H_
#define _SFC_GENERATOR_H_

#include <stdlib.h>
#include <cmath>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>

/*** polytopes utils  ***/
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
/*** polytopes utils  ***/

#include "sdlp.hpp"

namespace sfc_gen
{

  class SfcGenerator
  {
    // SECTION stable
  public:
    SfcGenerator(){}
    ~SfcGenerator(){}

    /* main planning interface */
    void initROS(ros::NodeHandle &nh)
    {
      /* read algorithm parameters */
      nh.param("plan_frame_id", frame_id_, std::string("world"));
      nh.param("optimization/ego_radius", ego_radius_, 0.2);
      /* sfc parameters */
      nh.param("sfc_gen/rils_bb_back", rils_bb_back_, 0.5);
      nh.param("sfc_gen/rils_bb_front", rils_bb_front_, 2.0);
      // sfc initialization
      Eigen::Vector3d map_origin, map_size;
      nh.param("map_ros/global/map_size_x", map_size(0), -1.0);
      nh.param("map_ros/global/map_size_y", map_size(1), -1.0);
      nh.param("map_ros/global/map_size_z", map_size(2), -1.0);
      nh.param("map_ros/global/map_origin_x", map_origin(0), -1.0);
      nh.param("map_ros/global/map_origin_y", map_origin(1), -1.0);
      nh.param("map_ros/global/map_origin_z", map_origin(2), -1.0);
      rils_poly_gen_.set_global_bbox(Vec3f(map_origin(0), map_origin(1), map_origin(2)),
                                    Vec3f(map_size(0), map_size(1), map_size(2)));  
      rils_poly_gen_.set_local_bbox(Vec3f(rils_bb_front_, 2.0, 1.0), Vec3f(rils_bb_back_, 2.0, 1.0));
      rils_poly_pub_   =  nh.advertise<decomp_ros_msgs::PolyhedronArray>("rils_polys", 1, true);

    }
   
  private:

    // ROS related
    std::string frame_id_;
    ros::Publisher rils_poly_pub_;

    //RILS 
    EllipsoidDecomp3D rils_poly_gen_;
    double rils_bb_back_, rils_bb_front_;
    double ego_radius_;
    std::vector<Eigen::MatrixXd> hPolys_;

    inline bool checkInsidePoly(Eigen::Vector3d &pt,
                         Eigen::MatrixXd &hPoly)
    {
      Eigen::Vector3d p_, n_;
      for (int i = 0; i < hPoly.cols(); i++){
          p_ = hPoly.col(i).head<3>();
          n_ = hPoly.col(i).tail<3>();
        
          if ( n_.dot(pt- p_) > 1e-6){return false;}
      }
      return true;
    }
    inline bool checkMAVInsidePoly(Eigen::Vector3d &pt,
                            Eigen::MatrixXd &hPoly)
    {
      unsigned int corr_k = hPoly.cols();
      for (unsigned int k = 0; k < corr_k; k++)
      {
          Eigen::Matrix<double, 3, 1> outerNormal = hPoly.col(k).tail<3>();
          double tempsfc = outerNormal.dot(pt - hPoly.col(k).head<3>());
          //std::cout << "rad_dis" << rad_dis << std::endl;
          if (tempsfc + ego_radius_ > 1e-6){
            return false;
          }
      }
      return true;

    }

    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        const int m = hPoly0.rows();
        const int n = hPoly1.rows();
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
        A.rightCols<1>().setConstant(1.0);
        b.topRows(m) = -hPoly0.rightCols<1>();
        b.bottomRows(n) = -hPoly1.rightCols<1>();
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    //poly is defined as h0*x + h1*y + h2*z + h3 <= 0 
    inline bool overlap(const Eigen::MatrixXd &hPoly0,
                        const Eigen::MatrixXd &hPoly1,
                        const double eps = 1.0e-6)

    {

        unsigned int m = hPoly0.cols();
        unsigned int n = hPoly1.cols();

        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);

        Eigen::MatrixX3d normals0 = (hPoly0.bottomRows<3>()).transpose();  //  (m, 3)
        Eigen::MatrixX3d normals1 = (hPoly1.bottomRows<3>()).transpose();  //  (n, 3)

        A.leftCols<3>().topRows(m) = normals0; // m * 3
        A.leftCols<3>().bottomRows(n) = normals1;
        A.rightCols<1>().setConstant(1.0);

        for (int i = 0; i < m; i ++)
        {
           b(i) =  normals0.row(i).dot(hPoly0.col(i).head<3>());
        }
        for (int j = 0; j < n; j ++)
        {
           b(m+j) =  normals1.row(j).dot(hPoly1.col(j).head<3>());
        }

        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

  public:
    typedef std::unique_ptr<SfcGenerator> Ptr;

  // SECTION interfaces for setup and query

    inline void getPolyConst(std::vector<Eigen::Vector3d> &path_pts,
                              double path_time_res,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr,
                              Eigen::MatrixXd &inner_pts,
                              Eigen::VectorXd &allo_ts,
                              std::vector<Eigen::MatrixXd> &hPolys)
    {

      hPolys.clear();

      Eigen::MatrixXd hPoly;
      std::vector<Eigen::Vector3d> temp_pts;
      std::vector<double> temp_ts;
      std::vector<Eigen::MatrixXd> temp_hPolys;


      //setup the pointcloud

      vec_Vec3f vec_obs;
      vec_obs.resize(cloud_ptr->points.size());
      for (unsigned int i = 0; i < cloud_ptr->points.size(); i++) {
        vec_obs[i](0) = cloud_ptr->points[i].x;
        vec_obs[i](1) = cloud_ptr->points[i].y;
        vec_obs[i](2) = cloud_ptr->points[i].z;
      }
      rils_poly_gen_.set_obs(vec_obs);

      
      size_t path_size = path_pts.size();
      Eigen::Vector3d q;
      int query_index, cnt_num = 0;
      std::cout << "the path_size is " << path_size << std::endl;
      // step 1 : set the intial lengh
      // add start time
      temp_ts.push_back(0.0);

      ros::Time time1= ros::Time::now();

      for (size_t i = 0; i < path_size-1; i++)
      {
        query_index = i;
        // check wehter or not we need to generate the point
        if (i > 0)
        {
          if (checkInsidePoly(path_pts[query_index], hPoly) && cnt_num <= 20)
          {
            cnt_num ++;
            continue;
          }
          else
          {
            //seed point is not inside
            query_index = i-1;
            cnt_num = 0;
          }
          temp_pts.push_back(path_pts[query_index]);
          temp_ts.push_back(query_index * path_time_res);
          
        }
        
        // get one polyhedron
        hPoly = getRILSConst(path_pts[query_index], path_pts[query_index+1]);
        temp_hPolys.push_back(hPoly);

      }
      
      size_t final_size = temp_pts.size(); // include the start and end point
      std::cout << "the final_size is " << final_size << std::endl;

      /***Corridor cut-off and refinements***/
      //1. if only one poly, add it
      if (final_size == 0){
        if(path_size==1)
        {
          temp_hPolys.push_back(hPoly);
        }
        temp_hPolys.push_back(hPoly);
        int temp_index = path_size/2;
        std::cout << "the path_size is " << path_size << std::endl;
        std::cout << "the temp_index is " << temp_index << std::endl;
        temp_pts.push_back(path_pts[temp_index]);
        temp_ts.push_back( temp_index* path_time_res);
        final_size = 1;
      }
      temp_ts.push_back(path_size * path_time_res);

      std::cout << "the temp_ts is " << temp_ts.size() << std::endl;
      std::cout << "the temp_hPolys.size() is " << temp_hPolys.size() << std::endl;

      //2. delete the overlap corridors
      int M = temp_hPolys.size();
      //if (M > 8){

        bool is_overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
          for (int j = 0; j < i; j++)
          {
            if (j < i - 1)
            {
              is_overlap = overlap(temp_hPolys[i], temp_hPolys[j], 0.01);
            }
            else
            {
              is_overlap = true;
            }
            if (is_overlap)
            {
              if (j < i - 1 && (!checkInsidePoly(temp_pts.at(j), temp_hPolys[i])))
              {
                continue;
              }
              idices.push_front(j);
              i = j + 1;
              break;

            }
          }
        }

        std::cout << "the idices. is " << idices.size() << std::endl;
        int short_cut_size = idices.size()-1;
        inner_pts.resize(3, short_cut_size);
        allo_ts.setZero(short_cut_size+1);
        hPolys.clear();

        int j = 0;
        int last_ele = 0;
        for (const auto &ele : idices)
        {
          hPolys.push_back(temp_hPolys[ele]);
          allo_ts(j) = temp_ts.at(ele+1) - temp_ts.at(last_ele);
          
          if (j < short_cut_size)
          {
            inner_pts.col(j) =  temp_pts.at(ele);
            std::cout << "htemp_pts.at(ele) is " << temp_pts.at(ele) << std::endl;
          }
          last_ele = ele+1;
          j +=1;
        }

      std::cout << "the sfc returns " << std::endl;
      hPolys_ = hPolys;
      return;
    }


    inline Eigen::MatrixXd getRILSConst(Eigen::Vector3d pt1, 
                                              Eigen::Vector3d pt2)
    {
      Eigen::MatrixXd hPoly;
      Vec3f seed_point1, seed_point2;
      vec_Vec3f seed_path;
      seed_point1 << pt1(0), pt1(1), pt1(2);
      seed_point2 << pt2(0), pt2(1), pt2(2);
      seed_path.push_back(seed_point1);
      seed_path.push_back(seed_point2);
      rils_poly_gen_.dilate(seed_path);
      hPoly = rils_poly_gen_.get_hPoly()[0];
      return hPoly;
    }

    inline void visPoly()
    {
      decomp_ros_msgs::PolyhedronArray poly_msg;
      for (const auto &hPoly : hPolys_)
      {
        decomp_ros_msgs::Polyhedron msg;
        geometry_msgs::Point pt, n;
        for (unsigned int i = 0; i < hPoly.cols(); i++)
        {  
          pt.x = hPoly(0, i);
          pt.y = hPoly(1, i);
          pt.z = hPoly(2, i);
          n.x  = hPoly(3, i);
          n.y  = hPoly(4, i);
          n.z  = hPoly(5, i);
          msg.points.push_back(pt);
          msg.normals.push_back(n);
        }
        poly_msg.polyhedrons.push_back(msg);
      }
      poly_msg.header.frame_id = frame_id_;
      rils_poly_pub_.publish(poly_msg);
      return;
    }

    // !SECTION
  };
} // namespace

#endif