#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    this->start_node=&m_Model.FindClosestNode(start_x,start_y);
    this->end_node=&m_Model.FindClosestNode(end_x,end_y);


}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return this->end_node->distance(*node);
}


float RoutePlanner::CalculateGValue(RouteModel::Node const *node) {
    return node->distance(*node->parent)+node->parent->g_value;
}
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->neighbors.clear();
    current_node->FindNeighbors();

    for (int i = 0; i < current_node->neighbors.size(); i ++)
    {
        RouteModel::Node *neighbor=current_node->neighbors[i];
        if(!neighbor->visited){
            neighbor->parent=current_node;
            neighbor->h_value=CalculateHValue(neighbor);
            neighbor->g_value=CalculateGValue(neighbor);
            neighbor->visited=true;
            open_list.push_back(neighbor);
        }
    }
    current_node->visited=true;
    
}

RouteModel::Node *RoutePlanner::NextNode() {
    struct {
        bool operator()(RouteModel::Node *a, RouteModel::Node *b) const { return a->g_value+a->h_value > b->g_value+b->h_value; }
    } much_to_less;
    std::sort(open_list.begin() , open_list.end() , much_to_less);
    RouteModel::Node *node_with_lowest_sum = open_list.back();
    open_list.pop_back();
    return node_with_lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node!=this->start_node)
    {
        path_found.push_back(*current_node);
        distance+=current_node->distance(*current_node->parent);
        current_node=current_node->parent;
    }

    path_found.push_back(*start_node);
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    while (current_node!=end_node)
    {

        AddNeighbors(current_node);
        current_node=NextNode();

    }
    m_Model.path=ConstructFinalPath(current_node);

}