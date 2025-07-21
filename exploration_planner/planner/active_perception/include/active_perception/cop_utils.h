#ifndef _COP_UTILS_H_
#define _COP_UTILS_H_

#include <ros/ros.h>

const int MAX_POINTS = 20;

namespace ms_planner {

struct Point {
  double x_;
  double y_;
  double center_x_;
  double center_y_;
};

bool clockwise_less(const Point& a, const Point& b);

double polygonArea(double X[], double Y[], int n);

double x_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4);

double y_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4);

void clip(double poly_points[][2], int &poly_size,
          double x1, double y1, double x2, double y2);

double suthHodgClip(double poly_points[][2], int poly_size,
                  double clipper_points[][2], int clipper_size);

}

#endif