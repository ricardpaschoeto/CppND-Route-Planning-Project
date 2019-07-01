#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int i = 0;
    for(auto &node : this->Nodes()){

        m_Nodes.push_back(Node(i, this, node));
        i++;
    }

    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap(){

    for(auto &road : Roads())
    {
        if(road.type != Model::Road::Type::Footway)
        {
            for(int idx: Ways()[road.way].nodes)
            {
                if(node_to_road.find(idx) == node_to_road.end())
                    node_to_road[idx] = std::vector<const Model::Road*> ();
                node_to_road[idx].push_back(&road);

            }
        }
    }

}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
    Node* closest_node = nullptr;
    Node node;
    for(int idx : node_indices)
    {
        node = parent_model->SNodes()[idx];
        if((!node.visited) && (distance(node)) != 0.0)
        {
            if(closest_node == nullptr || distance(node) < distance(*closest_node))
            {
                closest_node = &parent_model->SNodes()[idx];
            }

        }
    }

    return closest_node;
}

void RouteModel::Node::FindNeighbors(){

    for(auto &road: parent_model->node_to_road[this->index]){
        std::vector<int> node_indices = parent_model->Ways()[road->way].nodes;
        RouteModel::Node* closest_node = this->FindNeighbor(node_indices);
        if(closest_node != nullptr)
            this->neighbors.emplace_back(closest_node);

    }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y){
    
    Node node;

    node.x = x;
    node.y = y;
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    float d;

    for(auto &road : Roads()){
        if(road.type != Model::Road::Type::Footway){
            for(int idx : Ways()[road.way].nodes){
                d = node.distance(SNodes()[idx]);
                if(d < min_dist){
                    min_dist = d;
                    closest_idx = idx;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}
