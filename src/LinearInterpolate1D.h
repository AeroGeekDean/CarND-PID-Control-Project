/*
 * LinearInterpolate1D.h
 *
 * Found and copied this Linear (non-rectangular) interpolation routine online at:
 * https://stackoverflow.com/questions/11396860/better-way-than-if-else-if-else-for-linear-interpolation
 *
 *  Created on: Aug 11, 2017
 *      Author: deanliu
 */

#ifndef LINEARINTERPOLATE1D_H_
#define LINEARINTERPOLATE1D_H_

#include <utility>
#include <vector>
//#include <algorithm>

using std::pair;
using std::vector;

const double INF = 1.e100;

class LinearInterpolate1D
{
 public:

  /*
   * Constructor
   */
  LinearInterpolate1D();

  /*
   * Destructor
   */
  virtual ~LinearInterpolate1D();

  /*
   * Interpolate function
   */
  double interpolate(double x);

  /*
   * Add a point (x,y) to the table
   */
  void add_datapoints(pair<double, double> point_in);

 private:

  /*
   * Sort the table in ascending order, by .first
   */
  void sort_table();

  /*
   * Container holding the look-up table
   */
  vector<pair<double, double>> table;

};

/*****************************************************************************
 * Inline member functions                                                   *
 *****************************************************************************/

inline double LinearInterpolate1D::interpolate(double x)
{
  // Assumes that "table" is already sorted by .first

  // Check if x is out of bound
  // If so, return end point value
  if (x > table.back().first) return table.back().second;
  if (x < table[0].first) return table[0].second;

  vector<pair<double, double>>::iterator it;
  it = std::lower_bound(table.begin(),
                        table.end(),
                        std::make_pair(x, -INF));
  // Corner case
  if (it == table.begin()) return it->second;

  return (it-1)->second + (it->second-(it-1)->second)*(x-(it-1)->first)/(it->first-(it-1)->first);
}


inline void LinearInterpolate1D::add_datapoints(pair<double, double> point_in)
{
  table.push_back(point_in);
}


inline void LinearInterpolate1D::sort_table()
{
  std::sort(table.begin(),
            table.end(),
            [](const pair<double,double> &left, const pair<double,double> &right) // lambda function
              {
                return left.first < right.first;
              }
           );
}

#endif /* LINEARINTERPOLATE1D_H_ */
