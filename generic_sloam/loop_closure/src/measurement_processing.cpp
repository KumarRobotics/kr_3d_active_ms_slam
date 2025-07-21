#include "loop_closure/measurement_processing.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

bool in_region(const double& roll_vio, const double& pitch_vio,
               const std::vector<Eigen::Vector3d>& relative_msts,
               const std::vector<Eigen::Vector3d>& ugv_positions,
               std::vector<int>& matched_indices_out,
               Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out,
               Eigen::Matrix3d& best_rotation_matrix,
               const double& max_tol_per_landmark_pos_error) {
  Eigen::Matrix3d rotation_matrix_roll;
  // create a rotation matrix based on the roll angle
  rotation_matrix_roll << 1.0, 0.0, 0.0, 0.0, cos(roll_vio), -sin(roll_vio),
      0.0, sin(roll_vio), cos(roll_vio);

  Eigen::Matrix3d rotation_matrix_pitch;
  // create a rotation matrix based on the pitch angle
  rotation_matrix_pitch << cos(pitch_vio), 0.0, sin(pitch_vio), 0.0, 1.0, 0.0,
      -sin(pitch_vio), 0.0, cos(pitch_vio);

  Eigen::Matrix3d rotation_matrix_roll_pitch =
      rotation_matrix_roll * rotation_matrix_pitch;

  int i, j;

  int num_ugvs = ugv_positions.size();
  int num_msts = relative_msts.size();

  if (num_msts < num_ugvs) {
    return false;
  }

  int my_ints[num_msts];
  for (i = 0; i < num_msts; i++) {
    my_ints[i] = i;
  }

  int num_theta_candidates = 400;

  double min_residual = -1.0;

  do {
    // test a range of possible yaw angles
    for (i = 0; i < num_theta_candidates; i++) {
      double theta = M_PI * 2 * i / num_theta_candidates;

      Eigen::Matrix3d rotation_matrix_roll_pitch_yaw;
      Eigen::Matrix3d rotation_matrix_yaw;
      rotation_matrix_yaw << cos(theta), -sin(theta), 0.0, sin(theta),
          cos(theta), 0.0, 0.0, 0.0, 1.0;
      rotation_matrix_roll_pitch_yaw =
          rotation_matrix_roll_pitch * rotation_matrix_yaw;

      Eigen::Vector3d best_position_estimate;
      best_position_estimate << 0.0, 0.0, 0.0;
      for (j = 0; j < num_ugvs; j++) {
        best_position_estimate +=
            (ugv_positions[j] -
             rotation_matrix_roll_pitch_yaw * relative_msts[my_ints[j]]) /
            num_ugvs;
      }

      // now see what is the likelihood of actually making this set of
      // measurements given the sensing range
      double current_residual = 0.0;
      for (j = 0; j < num_ugvs; j++) {
        Eigen::Vector3d displacment =
            rotation_matrix_roll_pitch_yaw * relative_msts[my_ints[j]] +
            best_position_estimate - ugv_positions[j];
        current_residual += displacment.dot(displacment);
      }

      if ((current_residual < min_residual) || (min_residual < -0.5)) {
        min_residual = current_residual;
        yaw_estimate_out = theta;
        position_estimate_out = best_position_estimate;
        best_rotation_matrix = rotation_matrix_roll_pitch_yaw;
        for (j = 0; j < num_ugvs; j++) {
          matched_indices_out[j] = my_ints[j];
        }
      }
    }

  } while (std::next_permutation(my_ints, my_ints + num_msts));

  std::cout << "current min_residual: " << std::pow(min_residual, 0.5) << std::endl;
  if ((min_residual <= num_ugvs * max_tol_per_landmark_pos_error *
                           max_tol_per_landmark_pos_error)) {
    return true;
  } else {
    return false;
  }
}


bool estimate_bearingpos(const std::vector< Eigen::Vector3d>& measurements, const std::vector< Eigen::Vector3d>& obj_positions, bool fit_all_available,
              std::vector< std::array< int,2>>& matched_indices_out, Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out, int& exit_sign, const double& max_tol_per_landmark_pos_error)
{
  /*
  comments for input variables:
  - measurements: 
    list of relative position measurements to sensed objects in robot frame (unknown)
    we assume the robot has unknown positions and yaw, but know pitch and roll
  - obj_positions:
    positions of objects in the map with respect to the world frame 
  - fit_all_available:
    currently not in use 
    in future, if true, will require all measurements to be associated to some object in the map
  
  comments for output variables:
  - matched_indices_out:
    something goes here...:)
  - position_estimates_out:
    position estimate
  - yaw_estimate_out:
    estimate of the yaw angle
  - exit_sign:
    -2 if insufficient number of measurements
    0  if good, unambiguous localization 'achieved'
  */
  int i, j, k; //dummy index variables

  int num_msts = measurements.size();    //how many objects are we currently sensing
  int num_objs = obj_positions.size();   //how many objects are there in the map (or at least subset of the map we might be conceivably observing)
  if((num_msts < 3) || (num_objs < 3))   //if we sense less than three objects, we almost always have ambiguity so return FAILURE
  {
    exit_sign = -2; //insufficient number of objs or measurements
    return false;
  }


  std::vector<int> obj_combinations(num_objs);  //indices of objects we pick out in increasing index order
  std::vector<int> mst_combinations(num_msts);  //indices of measurements we pick out in increasing index order
  int permutation[num_msts];		              //which permutation of selected mst indices corresponds to selected object indices
  
  Eigen::Vector3d diff; //helper vector for storing intermediate differences between certain vectors
  double opt_loss = -1; //optimal loss, negative value less than -0.5 indicates PLUS infinity

  int num_matches;  //number of matches between sensed objects and those in the map
  for(num_matches = 4; (num_matches <= num_msts) && (num_matches <= num_objs); num_matches++)
  {
    
    for(i = 0; i < num_matches; i++)   //intialize first candidate combination of objects
    {
      obj_combinations[i] = i;
    }
    bool next_obj_comb_exists = true;   //keep searching through combinations of objects in lexicographic order 
    while( next_obj_comb_exists == true )
    {	
      //calculate the mean of obj's
      Eigen::Vector3d mean_obj = Eigen::Vector3d::Zero();
      for(i = 0; i < num_matches; i++)
      {
        mean_obj += obj_positions[ obj_combinations[i] ];
      }
      mean_obj /= num_matches;

      //calculate the fixed part of cost of obj's
      double obj_cost_fixed = 0;
      for(i = 0; i < num_matches; i++)
      {
        diff = obj_positions[ obj_combinations[i] ] - mean_obj;
        obj_cost_fixed += diff.dot(diff);
      }

      
      for(j = 0; j < num_matches; j++)   //initialize first candidate of combination of measurements
      {
        mst_combinations[j] = j;
      }
      bool next_mst_comb_exists = true;     //keep searching through combinations of measurements in lexicographic order 
      while( next_mst_comb_exists == true )
      {	
        //calculate the mean of mst's
        Eigen::Vector3d mean_mst = Eigen::Vector3d::Zero();
        for(i = 0; i < num_matches; i++)
        {
          mean_mst += measurements[ mst_combinations[i] ];
        }
        mean_mst /= num_matches;

        //calculate the fixed part of cost of obj's
        double mst_cost_fixed = 0;
        for(i = 0; i < num_matches; i++)
        {
          diff = measurements[ mst_combinations[i] ] - mean_mst;
          mst_cost_fixed += diff.dot(diff);
        }


        //now iterate through the possible permutations of matches and find the best one 
        for(i = 0; i < num_matches; i++)
        {
          permutation[i] = i;
        }

        do{

          Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
          for(i = 0; i < num_matches; i++)
          {
            correlation += (measurements[mst_combinations[i]] - mean_mst) * (obj_positions[obj_combinations[permutation[i]]] - mean_obj).transpose();
          }

          double cos_coeff = 0;
          double sin_coeff = 0;
          double const_coeff = 0;

          cos_coeff = correlation(0,0) + correlation(1,1);
          sin_coeff = correlation(0,1) - correlation(1,0);
          const_coeff = correlation(2,2);

          double coeff_strength = sqrt(cos_coeff * cos_coeff + sin_coeff * sin_coeff);

          double temp_loss = obj_cost_fixed + mst_cost_fixed - 2 * (coeff_strength + const_coeff);
          temp_loss /= num_matches;

          if((opt_loss < -0.5) || (temp_loss < opt_loss))
          {
            opt_loss = temp_loss;

            yaw_estimate_out = atan2(sin_coeff/coeff_strength, cos_coeff/coeff_strength);

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << cos(yaw_estimate_out), -sin(yaw_estimate_out), 0.0,
                      sin(yaw_estimate_out), cos(yaw_estimate_out), 0.0, 
                      0.0, 0.0, 1.0;
            position_estimate_out = mean_obj - rotation_matrix * mean_mst;

            matched_indices_out.clear();
            for(k = 0; k < num_matches; k++)
            {
              std::array<int, 2> new_match = { mst_combinations[k], obj_combinations[permutation[k]] };
              matched_indices_out.push_back( new_match );
              std::cout << "Msts: k:" << k << " " << measurements[mst_combinations[k]][0] << " " << measurements[mst_combinations[k]][1] << " " << measurements[mst_combinations[k]][2] << std::endl;

            }
            std::cout << "Update msts" << std::endl;

          }

        } while(std::next_permutation(permutation, permutation + num_matches));



        //find lexicographically subsequent mst combination
        j = num_matches - 1;
        next_mst_comb_exists = false;
        while( (j >= 0) && (next_mst_comb_exists == false) )
        {	
          if( mst_combinations[j] == (num_msts - 1) + j - (num_matches - 1) )
          {
            --j;
          }
          else
          {
            next_mst_comb_exists = true;
          }
        }
        if(next_mst_comb_exists == true)
        {
          ++mst_combinations[j];
          for(k = j+1; k < num_matches; k++)
          {
            mst_combinations[k] = mst_combinations[k-1] + 1;
          }
        }
      }


      //find lexicographically subsequent obj combination
      j = num_matches - 1;
      next_obj_comb_exists = false;
      while( (j >= 0) && (next_obj_comb_exists == false) )
      {	
        if( obj_combinations[j] == (num_objs - 1) + j - (num_matches - 1) )
        {
          --j;
        }
        else
        {
          next_obj_comb_exists = true;
        }
      }
      if(next_obj_comb_exists == true)
      {
        ++obj_combinations[j];
        for(k = j+1; k < num_matches; k++)
        {
          obj_combinations[k] = obj_combinations[k-1] + 1;
        }
      }
    }

  }

  //wrap up
  exit_sign = 0;
  std::cout << "current min_residual 1: " << std::pow(opt_loss,0.5) << std::endl;
  if ((opt_loss <= max_tol_per_landmark_pos_error * max_tol_per_landmark_pos_error)) {
    return true;
  } else {
    return false;
  }

}



// Interface that takes Yaw prior
bool estimate_bearingpos(const std::vector< Eigen::Vector3d>& measurements, const std::vector< Eigen::Vector3d>& obj_positions, bool fit_all_available, double yaw_center, double yaw_max_dev, 
              std::vector< std::array< int,2>>& matched_indices_out, Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out, int& exit_sign, const double& max_tol_per_landmark_pos_error)
{
  /*
  comments for input variables:
  - measurements: 
    list of relative position measurements to sensed objects in robot frame (unknown)
    we assume the robot has unknown positions and yaw, but know pitch and roll
  - obj_positions:
    positions of objects in the map with respect to the world frame 
  - fit_all_available:
    currently not in use 
    in future, if true, will require all measurements to be associated to some object in the map
  - yaw_center: 
    central yaw angle 
  - yaw_max_dev: 
    maximum possible aniticipated deviation of actual yaw from yaw_center
  
  comments for output variables:
  - matched_indices_out:
    something goes here...:)
  - position_estimates_out:
    position estimate
  - yaw_estimate_out:
    estimate of the yaw angle
  - exit_sign:
    -2 if insufficient number of measurements
    0  if good, unambiguous localization 'achieved'
  */
  int i, j, k; //dummy index variables


  int num_msts = measurements.size();    //how many objects are we currently sensing
  int num_objs = obj_positions.size();   //how many objects are there in the map (or at least subset of the map we might be conceivably observing)
  if((num_msts < 3) || (num_objs < 3))   //if we sense less than three objects, we almost always have ambiguity so return FAILURE
  {
    exit_sign = -2; //insufficient number of objs or measurements
    return false;
  }

  std::vector<int> obj_combinations(num_objs);  //indices of objects we pick out in increasing index order
  std::vector<int> mst_combinations(num_msts);  //indices of measurements we pick out in increasing index order
  int permutation[num_msts];		              //which permutation of selected mst indices corresponds to selected object indices
  
  Eigen::Vector3d diff; //helper vector for storing intermediate differences between certain vectors
  double opt_loss = -1; //optimal loss, negative value less than -0.5 indicates PLUS infinity

  int num_matches;  //number of matches between sensed objects and those in the map
  // for(num_matches = 3; (num_matches <= num_msts) && (num_matches <= num_objs); num_matches++)
  for(num_matches = std::min(num_msts, num_objs); num_matches >= 3; num_matches--)
  {
    
    for(i = 0; i < num_matches; i++)   //intialize first candidate combination of objects
    {
      obj_combinations[i] = i;
    }
    bool next_obj_comb_exists = true;   //keep searching through combinations of objects in lexicographic order 
    while( next_obj_comb_exists == true )
    {	
      //calculate the mean of obj's
      Eigen::Vector3d mean_obj = Eigen::Vector3d::Zero();
      for(i = 0; i < num_matches; i++)
      {
        mean_obj += obj_positions[ obj_combinations[i] ];
      }
      mean_obj /= num_matches;

      //calculate the fixed part of cost of obj's
      double obj_cost_fixed = 0;
      for(i = 0; i < num_matches; i++)
      {
        diff = obj_positions[ obj_combinations[i] ] - mean_obj;
        obj_cost_fixed += diff.dot(diff);
      }

      
      for(j = 0; j < num_matches; j++)   //initialize first candidate of combination of measurements
      {
        mst_combinations[j] = j;
      }
      bool next_mst_comb_exists = true;     //keep searching through combinations of measurements in lexicographic order 
      while( next_mst_comb_exists == true )
      {	
        //calculate the mean of mst's
        Eigen::Vector3d mean_mst = Eigen::Vector3d::Zero();
        for(i = 0; i < num_matches; i++)
        {
          mean_mst += measurements[ mst_combinations[i] ];
        }
        mean_mst /= num_matches;

        //calculate the fixed part of cost of obj's
        double mst_cost_fixed = 0;
        for(i = 0; i < num_matches; i++)
        {
          diff = measurements[ mst_combinations[i] ] - mean_mst;
          mst_cost_fixed += diff.dot(diff);
        }


        //now iterate through the possible permutations of matches and find the best one 
        for(i = 0; i < num_matches; i++)
        {
          permutation[i] = i;
        }

        do{

          Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
          for(i = 0; i < num_matches; i++)
          {
            correlation += (measurements[mst_combinations[i]] - mean_mst) * (obj_positions[obj_combinations[permutation[i]]] - mean_obj).transpose();
          }

          double cos_coeff = 0;
          double sin_coeff = 0;
          double const_coeff = 0;

          cos_coeff = correlation(0,0) + correlation(1,1);
          sin_coeff = correlation(0,1) - correlation(1,0);
          const_coeff = correlation(2,2);

          double coeff_strength = sqrt(cos_coeff * cos_coeff + sin_coeff * sin_coeff);
          double yaw_temp = atan2(sin_coeff/coeff_strength, cos_coeff/coeff_strength);

          Eigen::Vector3d yaw_center_heading;
          yaw_center_heading << cos(yaw_center), sin(yaw_center), 0;

          Eigen::Vector3d yaw_temp_heading;
          yaw_temp_heading << cos_coeff / coeff_strength, sin_coeff / coeff_strength, 0;

          double relative_angle = acos( yaw_temp_heading.dot(yaw_center_heading) );
          double cross_product = yaw_center_heading.cross(yaw_temp_heading)[2];

          double yaw_candidate;

          if(cross_product >= 0)
          {
            if(relative_angle > yaw_max_dev)
            {
              yaw_candidate = yaw_center + yaw_max_dev;
              if(yaw_candidate > M_PI)
              {
                yaw_candidate -= 2*M_PI;
              }
            }
            else
            {
              yaw_candidate = yaw_temp;
            }
          }
          else
          {
            if(relative_angle > yaw_max_dev)
            {
              yaw_candidate = yaw_center - yaw_max_dev;
              if(yaw_candidate < -M_PI)
              {
                yaw_candidate += 2*M_PI;
              }
            }
            else
            {
              yaw_candidate = yaw_temp;
            }
          }


          double temp_loss = obj_cost_fixed + mst_cost_fixed - 2 * (coeff_strength * cos(yaw_candidate - yaw_temp) + const_coeff);
          temp_loss /= num_matches;

          if((opt_loss < -0.5) || (temp_loss < opt_loss))
          {
            opt_loss = temp_loss;

            yaw_estimate_out = yaw_candidate;

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << cos(yaw_estimate_out), -sin(yaw_estimate_out), 0.0,
                      sin(yaw_estimate_out), cos(yaw_estimate_out), 0.0, 
                      0.0, 0.0, 1.0;
            position_estimate_out = mean_obj - rotation_matrix * mean_mst;

            matched_indices_out.clear();
            for(k = 0; k < num_matches; k++)
            {
              std::array<int, 2> new_match = { mst_combinations[k], obj_combinations[permutation[k]] };
              matched_indices_out.push_back( new_match );
              std::cout << "Msts: k:" << k << " " << measurements[mst_combinations[k]][0] << " " << measurements[mst_combinations[k]][1] << " " << measurements[mst_combinations[k]][2] << std::endl;

            }
            std::cout << "Update msts" << std::endl;

            if (opt_loss <= max_tol_per_landmark_pos_error * max_tol_per_landmark_pos_error) {
              std::cout << "current min_residual 2: " << std::pow(opt_loss,0.5) << std::endl;
              return true;
            }
          }

        } while(std::next_permutation(permutation, permutation + num_matches));



        //find lexicographically subsequent mst combination
        j = num_matches - 1;
        next_mst_comb_exists = false;
        while( (j >= 0) && (next_mst_comb_exists == false) )
        {	
          if( mst_combinations[j] == (num_msts - 1) + j - (num_matches - 1) )
          {
            --j;
          }
          else
          {
            next_mst_comb_exists = true;
          }
        }
        if(next_mst_comb_exists == true)
        {
          ++mst_combinations[j];
          for(k = j+1; k < num_matches; k++)
          {
            mst_combinations[k] = mst_combinations[k-1] + 1;
          }
        }
      }


      //find lexicographically subsequent obj combination
      j = num_matches - 1;
      next_obj_comb_exists = false;
      while( (j >= 0) && (next_obj_comb_exists == false) )
      {	
        if( obj_combinations[j] == (num_objs - 1) + j - (num_matches - 1) )
        {
          --j;
        }
        else
        {
          next_obj_comb_exists = true;
        }
      }
      if(next_obj_comb_exists == true)
      {
        ++obj_combinations[j];
        for(k = j+1; k < num_matches; k++)
        {
          obj_combinations[k] = obj_combinations[k-1] + 1;
        }
      }
    }

  }
  //wrap up
  exit_sign = 0;
  std::cout << "current min_residual 2: " << std::pow(opt_loss,0.5) << std::endl;
  if ((opt_loss <= max_tol_per_landmark_pos_error * max_tol_per_landmark_pos_error)) {
    return true;
  } else {
    return false;
  }
}