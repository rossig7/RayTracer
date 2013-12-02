#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <queue>
#include <mutex>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <thread>
#include <random>

#define BVH_SELF_TEST_TIMES 0
#define BVH_PERF_TEST 0
#define KD_SELF_TEST_TIMES 0
#define KD_PERF_TEST 0
#define MULTI_THREAD 1

static std::random_device rd;
static std::mt19937 gen(rd());

using namespace std;

#endif