#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() { return distance; }
    void AStarSearch();

  private:
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node* current_node);
    float CalculateHValue(const RouteModel::Node* node);
    RouteModel::Node* NextNode();
    void AddNeighbors(RouteModel::Node* node);
    
  private:
    RouteModel &m_Model;
    RouteModel::Node* start_node;
    RouteModel::Node* end_node;
    float distance;
    std::vector<RouteModel::Node*> open_list;
};
