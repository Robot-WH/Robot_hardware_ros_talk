#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Dense>

//#include <glog/logging.h>

using namespace std;

constexpr double CoefDegreeToRadian = M_PI / 180.;
constexpr double CoefRadianToDegree = 180. / M_PI;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取反对称矩阵 
static Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}


#endif
