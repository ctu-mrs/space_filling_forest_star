/**
 * @file main.cpp
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 25/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "main.h"

int main(int argc, char *argv[]) {
  if (argc < 2) {
    return 2;
  }
  
  Problem<double> problem;
  if (argc == 3) {
    problem.iteration = std::stoi(argv[2]);
  }
  parseFile(std::string(argv[1]), problem);
  std::unique_ptr<Solver<double, Point<double>>> solver;
  if (problem.solver == SFF) {
    solver = std::make_unique<SpaceForest<double, Point<double>>>(problem);  
  } else if (problem.solver == RRT) {
    solver = std::make_unique<RapidExpTree<double, Point<double>>>(problem);
  } else if (problem.solver == Lazy) { 
    solver = std::make_unique<LazyTSP<double, Point<double>>>(problem);
  } else {
    throw std::string("Not implemented!");
  }

  solver->Solve();
  
  return 0;
}

void parseFile(const std::string &fileName, Problem<double> &problem) {
  rapidxml::xml_document<> config;
  char *pString;
  double scale;
  Point<double> pos;
	std::ifstream fileStream(fileName.c_str());
	if (!fileStream.good()) {
		std::cout << "Cannot open config file at: " << fileName << "\n";
		exit(1);
	}

	if (fileStream.is_open()) {
		std::stringstream content;
		content << fileStream.rdbuf();
    pString = strdup(content.str().c_str());
		config.parse<0>(pString);
		fileStream.close();
	} else {
		std::cout << "Cannot open config file at: " << fileName << "\n";
		exit(1);
	}

  try {
    rapidxml::xml_node<> *rootNode{config.first_node("Problem")};
    rapidxml::xml_attribute<> *attr;
    if (rootNode == nullptr) {
      throw std::invalid_argument("invalid root node");
    }

    attr = rootNode->first_attribute("solver");
    if (attr == nullptr) {
      throw std::invalid_argument("invalid solver attibute in Problem node!");
    }
    if (!strcmp(attr->value(), "sff")) {
      problem.solver = SFF;
    } else if (!strcmp(attr->value(), "rrt")) {
      problem.solver = RRT;
    } else if (!strcmp(attr->value(), "lazy")) {
      problem.solver = Lazy;
    } else {
      throw std::invalid_argument("unknown solver type in Problem node, use either sff or rrt");
    }

    attr = rootNode->first_attribute("optimize");
    if (attr == nullptr) {
      throw std::invalid_argument("invalid optimize attibute in Problem node!");
    }
    problem.optimal = (!strcmp(attr->value(), "true"));

    attr = rootNode->first_attribute("smoothing");
    problem.smoothing = (attr != nullptr && !strcmp(attr->value(), "true"));
    if (problem.solver == Lazy && problem.smoothing) {
      throw std::invalid_argument("Lazy-RRT* solver with path smoothing is not implemented, set \"smoothing\" parameter to \"false\"");
    }

    attr = rootNode->first_attribute("scale");
    if (attr == nullptr) {
      scale = 1;
    } else {
      scale = std::stod(attr->value());
    }

    attr = rootNode->first_attribute("dim");
    if (attr == nullptr || !strcmp(attr->value(), "3D") || !strcmp(attr->value(), "3d")) {
      problem.dimension = D3;
    } else if (!strcmp(attr->value(), "2D") || !strcmp(attr->value(), "2d")) {
      problem.dimension = D2;
    } else {
      throw std::invalid_argument("invalid dim attribute!");
    }

    // parse delimiters
    std::string delimiter{" "}, nameDelimiter{"_"};
    rapidxml::xml_node<> *node{rootNode->first_node("ObjectDelimiters")};
    rapidxml::xml_node<> *subNode;

    if (node != nullptr) {
      attr = node->first_attribute("standard");
      if (attr != nullptr) {
        Obstacle<double>::Delimiter = attr->value(); 
      }

      attr = node->first_attribute("name");
      if(attr != nullptr) {
        Obstacle<double>::NameDelimiter = attr->value();
      }
    }

    // TSP parameters for Lazy solver
    node = rootNode->first_node("TSP");
    if (node == nullptr && problem.solver == Lazy) {
      throw std::invalid_argument("missing TSP solver parameters for Lazy solver!");
    } else if (node != nullptr) {
      if (problem.solver != Lazy) {
        std::cout << "Warning: TSP solver is called only in Lazy solver algorithm -- defined TSP parameters are redundant and will not be used.\n";
      }
      
      attr = node->first_attribute("path");
      if (attr == nullptr) {
        throw std::invalid_argument("invalid path attribute in TSP node!");
      }
      problem.tspSolver = attr->value();

      attr = node->first_attribute("type");
      if (attr == nullptr) {
        throw std::invalid_argument("invalid type attribute in TSP node!");
      }
      problem.tspType = attr->value();
    }


    // parse Robot
    node = rootNode->first_node("Robot");
    if (node == nullptr) {
      throw std::invalid_argument("invalid Robot node!");
    }
    FileStruct tempFile;
    if (getFile(node, tempFile)) {
      throw std::invalid_argument("invalid file node in Robot node!");
    }

    problem.environment.Robot = new Obstacle<double>(tempFile.fileName, tempFile.type == Obj, scale);

    // parse Range
    node = rootNode->first_node("Range");
    if (node == nullptr) {
      throw std::invalid_argument("invalid range node");
    }

    attr = node->first_attribute("autoDetect");
    problem.autoRange = (attr != nullptr && !strcmp(attr->value(), "true"));
    
    if (!problem.autoRange) {
      subNode = node->first_node("RangeX");
      if (subNode == nullptr) {
        throw std::invalid_argument("invalid rangex node in range node");
      }
      attr = subNode->first_attribute("min");
      if (attr == nullptr) {
        throw invalid_argument("invalid min attribute in rangex node");
      }
      problem.environment.limits.minX = scale * std::stod(attr->value());

      attr = subNode->first_attribute("max");
      if (attr == nullptr) {
        throw invalid_argument("invalid max attribute in rangex node");
      }
      problem.environment.limits.maxX = scale * std::stod(attr->value());

      subNode = node->first_node("RangeY");
      if (subNode == nullptr) {
        throw std::invalid_argument("invalid rangey node in range node");
      }
      attr = subNode->first_attribute("min");
      if (attr == nullptr) {
        throw invalid_argument("invalid min attribute in rangey node");
      }
      problem.environment.limits.minY = scale * std::stod(attr->value());

      attr = subNode->first_attribute("max");
      if (attr == nullptr) {
        throw invalid_argument("invalid max attribute in rangey node");
      }
      problem.environment.limits.maxY = scale * std::stod(attr->value());

      subNode = node->first_node("RangeZ");
      if (subNode == nullptr) {
        throw std::invalid_argument("invalid rangez node in range node");
      }
      attr = subNode->first_attribute("min");
      if (attr == nullptr) {
        throw invalid_argument("invalid min attribute in rangez node");
      }
      problem.environment.limits.minZ = scale * std::stod(attr->value());

      attr = subNode->first_attribute("max");
      if (attr == nullptr) {
        throw invalid_argument("invalid max attribute in rangez node");
      }
      problem.environment.limits.maxZ = scale * std::stod(attr->value());
    }

    // parse environment
    node = rootNode->first_node("Environment");
    if (node == nullptr) {
      problem.environment.HasMap = false;  
    } else {
      attr = node->first_attribute("collision");
      if (attr == nullptr) {
        throw std::invalid_argument("invalid collision attribute in Environment node!");
      }
      problem.collisionDist = scale * std::stod(attr->value());

      problem.environment.ScaleFactor = scale;
      
      subNode = node->first_node("Obstacle");
      if (subNode == nullptr) {
        problem.environment.HasMap = false;
      }

      while (subNode != nullptr) {
        FileStruct file;

        if (getFile(subNode, file)) {
          throw std::invalid_argument("invalid file attribute in Obstacle node!");
        }

        attr = subNode->first_attribute("position");
        if (attr == nullptr) {
          pos = Point<double>();
        } else {
          pos = Point<double>(attr->value());
        }

        Obstacle<double> &obst{problem.environment.Obstacles.emplace_back(file.fileName, 
          file.type == Obj, pos, scale)};

        if (problem.autoRange) {
          problem.environment.processLimits(obst.getRange());
        }

        subNode = subNode->next_sibling();
      }
    }
    
    // parse start points
    node = rootNode->first_node("Points");
    if (node == nullptr) {
      throw std::invalid_argument("invalid Points node - insert at least one point!");  
    } else {
      int numPoints{0};
      subNode = node->first_node("Point");
      if (subNode == nullptr) {
        throw std::invalid_argument("invalid Point subnode in Points node - insert at least one point!");
      }
      while (subNode != nullptr) {
        attr = subNode->first_attribute("coord");
        if (attr == nullptr) {
          throw std::invalid_argument("invalid coord attribute in Point node!");
        }
        problem.roots.emplace_back(attr->value(), scale);

        subNode = subNode->next_sibling();
        ++numPoints;
      }
      
      if (problem.solver == RRT && problem.optimal && numPoints > 1) {
        throw std::invalid_argument("Multi-T-RRT* is undefined!");
      } 
    }

    node = rootNode->first_node("Goal");
    if (node != nullptr) {
      if (problem.solver == Lazy) {
        throw std::invalid_argument("single point path planning not defined for Lazy solver (use RRT/RRT* solver instead)!");
      } else if (problem.roots.size() > 1) {
        std::cout << "Warning: Multi-source planning with one goal has not been tested!\n";
      }
      problem.hasGoal = true;
      attr = node->first_attribute("coord");
      if (attr == nullptr) {
        throw std::invalid_argument("invalid coord attribute in Goal node!");
      }
      problem.goal = Point<double>(attr->value(), scale);
    }

    // parse Distances
    node = rootNode->first_node("Distances");
    if (node == nullptr) {
      throw std::invalid_argument("invalid Distances node!");
    }
    attr = node->first_attribute("dtree");
    if (attr == nullptr) {
      throw std::invalid_argument("invalid dtree attribute in Distances node!");
    }
    problem.distTree = scale * std::stod(attr->value());
    
    attr = node->first_attribute("circum");
    if (attr == nullptr) {
      throw std::invalid_argument("invalid circum attribute in Distances node!");
    }
    Node<double, Point<double>>::SamplingDistance = scale * std::stod(attr->value());

    // improvements
    node = rootNode->first_node("Improvements");
    if (node != nullptr) {
      attr = node->first_attribute("priorityBias");
      if (attr != nullptr) {
        problem.priorityBias = std::stod(attr->value());
      } 
      if (!problem.hasGoal && problem.priorityBias != 0 && problem.solver == RRT) {
        throw std::invalid_argument("Multi-T-RRT with bias is undefined!");
      } else if (problem.solver == Lazy && problem.priorityBias != 0) {
        throw std::invalid_argument("priority bias for Lazy solver is not implemented!");
      }
    } 

    // thresholds
    node = rootNode->first_node("Thresholds");
    if (node != nullptr) {
      attr = node->first_attribute("standard");
      if (attr != nullptr) {
        Node<double>::ThresholdMisses = std::stoi(attr->value());
      }
    }

    // max iterations
    node = rootNode->first_node("MaxIterations");
    if (node == nullptr) {
      throw std::invalid_argument("invalid MaxIterations node");
    }
    attr = node->first_attribute("value");
    if (attr == nullptr) {
      throw std::invalid_argument("invalid value attribute in MaxIterations node");
    }
    problem.maxIterations = std::stoi(attr->value());    

    // save options
    node = rootNode->first_node("Save");
    if (node != nullptr) {
      FileStruct tempFile;
      subNode = node->first_node("Goals");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        problem.saveOptions = problem.saveOptions | SaveGoals;
        problem.fileNames[SaveGoals] = tempFile;
      }

      subNode = node->first_node("Tree");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        problem.saveOptions = problem.saveOptions | SaveTree;
        problem.fileNames[SaveTree] = tempFile;

        attr = subNode->first_attribute("everyIteration");
        if (attr != nullptr && std::stoi(attr->value()) != 0) {
          problem.saveOptions = problem.saveOptions | SaveConcurrent;
          problem.saveTreeIter = std::stoi(attr->value());
        }
      }

      subNode = node->first_node("RawPath");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        problem.saveOptions = problem.saveOptions | SaveRaw;
        problem.fileNames[SaveRaw] = tempFile;
      }

      subNode = node->first_node("SmoothPath");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        if (problem.smoothing) {
          throw std::invalid_argument("smoothing is disabled, therefore \"SmoothPath\" parameter might not be defined!");
        }
        problem.saveOptions = problem.saveOptions | SaveSmooth;
        problem.fileNames[SaveSmooth] = tempFile;
      }

      subNode = node->first_node("Params");
      if (!getFile(subNode, tempFile, problem.iteration, false)) {
        problem.saveOptions = problem.saveOptions | SaveParams;
        problem.fileNames[SaveParams] = tempFile;
        
        attr = subNode->first_attribute("id");
        if (attr != nullptr) {
          problem.id = attr->value();
        }
      }

      subNode = node->first_node("TSP");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        problem.saveOptions = problem.saveOptions | SaveTSP;
        problem.fileNames[SaveTSP] = tempFile;
      }

      subNode = node->first_node("Frontiers");
      if (!getFile(subNode, tempFile, problem.iteration)) {
        if (problem.solver != SFF) {
          throw std::invalid_argument("frontiers output is defined only for SFF-based solvers!");
        }
        problem.saveOptions = problem.saveOptions | SaveFrontiers;
        problem.fileNames[SaveFrontiers] = tempFile;

        attr = subNode->first_attribute("everyIteration");
        if (attr != nullptr && std::stoi(attr->value()) != 0) {
          problem.saveOptions = problem.saveOptions | SaveConcurrent;
          problem.saveFrontiersIter = std::stoi(attr->value());
        }
      }
    }

  } catch (const std::invalid_argument &e) {
    std::cout << "Problem loading error: " << e.what() << "\n";
    if (pString != NULL) {
      free(pString);
    }
    exit(1);
  }
	if (pString != NULL) {
    free(pString);
  }
}

bool getFile(rapidxml::xml_node<> *node, FileStruct &file, int iteration, bool includeIter) {
  rapidxml::xml_attribute<> *attr;
  if (node == nullptr) {
    return true;
  }
  
  attr = node->first_attribute("file");
  if (attr == nullptr) {
    return true;
  }  
  file.fileName = attr->value();
  if (iteration != 0 && includeIter) {
    size_t iter{file.fileName.find_last_of('.')};
    file.fileName.insert(iter, '_' + std::to_string(iteration));
  }

  attr = node->first_attribute("is_obj");
  if (attr == nullptr || !strcmp(attr->value(), "false")) {
    file.type = Map;
  } else if (!strcmp(attr->value(), "true")) {
    file.type = Obj;
  } else {
    throw std::invalid_argument("invalid attribute isObj in file node!");
  }

  return false;
}
