#include "active_perception/cop_utils.h"

namespace ms_planner {

bool clockwise_less(const Point& a, const Point& b) {
    if (a.x_ - a.center_x_ >= 0 && b.x_ - a.center_x_ < 0)
        return true;
    if (a.x_ - a.center_x_ < 0 && b.x_ - a.center_x_ >= 0)
        return false;
    if (a.x_ - a.center_x_ == 0 && b.x_ - a.center_x_ == 0) {
        if (a.y_ - a.center_y_ >= 0 || b.y_ - a.center_y_ >= 0)
            return a.y_ > b.y_;
        return b.y_ > a.y_;
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    double det = (a.x_ - a.center_x_) * (b.y_ - a.center_y_) - (b.x_ - a.center_x_) * (a.y_ - a.center_y_);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    double d1 = (a.x_ - a.center_x_) * (a.x_ - a.center_x_) + (a.y_ - a.center_y_) * (a.y_ - a.center_y_);
    double d2 = (b.x_ - a.center_x_) * (b.x_ - a.center_x_) + (b.y_ - a.center_y_) * (b.y_ - a.center_y_);
    return d1 > d2;
}

// (X[i], Y[i]) are coordinates of i'th point.
double polygonArea(double X[], double Y[], int n) {
    // Initialize area
    double area = 0.0;
 
    // Calculate value of shoelace formula
    int j = n - 1;
    for (int i = 0; i < n; i++)
    {
        area += (X[j] + X[i]) * (Y[j] - Y[i]);
        j = i;  // j is previous vertex to i
    }
 
    // Return absolute value
    return abs(area / 2.0);
}

// Returns x-value of point of intersection of two
// lines
double x_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4) {
    double num = (x1*y2 - y1*x2) * (x3-x4) -
              (x1-x2) * (x3*y4 - y3*x4);
    double den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}


// Returns y-value of point of intersection of
// two lines
double y_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4) {
    double num = (x1*y2 - y1*x2) * (y3-y4) -
              (y1-y2) * (x3*y4 - y3*x4);
    double den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}


// This functions clips all the edges w.r.t one clip
// edge of clipping area
void clip(double poly_points[][2], int &poly_size,
          double x1, double y1, double x2, double y2) {
    double new_points[MAX_POINTS][2];
    int new_poly_size = 0;
  
    // (ix,iy),(kx,ky) are the co-ordinate values of
    // the points
    for (int i = 0; i < poly_size; i++) {
        // i and k form a line in polygon
        int k = (i+1) % poly_size;
        double ix = poly_points[i][0], iy = poly_points[i][1];
        double kx = poly_points[k][0], ky = poly_points[k][1];
        // Calculating position of first point
        // w.r.t. clipper line
        double i_pos = (x2-x1) * (iy-y1) - (y2-y1) * (ix-x1);
        // Calculating position of second point
        // w.r.t. clipper line
        double k_pos = (x2-x1) * (ky-y1) - (y2-y1) * (kx-x1);
        // Case 1 : When both points are inside
        if (i_pos < 0  && k_pos < 0) {
            //Only second point is added
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 2: When only first point is outside
        else if (i_pos >= 0  && k_pos < 0) {
            // Point of intersection with edge
            // and the second point is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
  
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 3: When only second point is outside
        else if (i_pos < 0  && k_pos >= 0) {
            //Only point of intersection with edge is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
        }
  
        // Case 4: When both points are outside
        else {
            //No points are added
        }
    }
  
    // Copying new points into original array
    // and changing the no. of vertices
    poly_size = new_poly_size;
    for (int i = 0; i < poly_size; i++)
    {
        poly_points[i][0] = new_points[i][0];
        poly_points[i][1] = new_points[i][1];
    }
}
  
// Implements Sutherland–Hodgman algorithm
double suthHodgClip(double poly_points[][2], int poly_size,
                  double clipper_points[][2], int clipper_size) {
    //i and k are two consecutive indexes
    for (int i=0; i<clipper_size; i++) {
        int k = (i+1) % clipper_size;
  
        // We pass the current array of vertices, it's size
        // and the end points of the selected clipper line
        clip(poly_points, poly_size, clipper_points[i][0],
             clipper_points[i][1], clipper_points[k][0],
             clipper_points[k][1]);
    }
    if (poly_size == 0) {
        return 0;
    } else {
        double X[poly_size];
        double Y[poly_size];
        for (int j = 0; j < poly_size; j++) {
            X[j] = poly_points[j][0];
            Y[j] = poly_points[j][1];
        }
        return polygonArea(X, Y, poly_size);
    }

}


} // namespace ms_planner