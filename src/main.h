/**
 * @file main.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 25/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include "rapidxml.hpp"

#include "primitives.h"
#include "environment.h"
#include "problemStruct.h"
#include "forest.h"
#include "rrt.h"
#include "lazy.h"

using namespace std;
using namespace rapidxml;

void parseFile(const std::string &fileName, Problem<double> &problem);
bool getFile(rapidxml::xml_node<> *node, FileStruct &file, int iteration = 0, bool includeIter = true);

#endif
