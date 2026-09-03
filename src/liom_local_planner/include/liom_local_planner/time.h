//
// Created by 欧阳亚坤 on 2021/9/15.
//
//#pragma once
#ifndef LIOM_LOCAL_PLANNER_TIME_H
#define LIOM_LOCAL_PLANNER_TIME_H

#include <chrono>

namespace liom_local_planner {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
  return ((double) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count() / 1000);
}


}
#endif