#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*this->end_node);
    return h_value;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    //RouteModel::Node *node;
    for (auto* node: current_node->neighbors){
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);

        this->open_list.push_back(node);
        node->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

// create a function to compare nodes condition
bool CompareNodes(const RouteModel::Node* a, const RouteModel::Node* b) {
     
    return (a->h_value + a->g_value) < (b->h_value + b->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Initialize the next_node
    RouteModel::Node *next_node = nullptr;

    // before mentor help
    //next_node->h_value = std::numeric_limits<float>::max();
    //next_node->g_value = std::numeric_limits<float>::max();
    // // Find the node in open_list that has smallest f valude
    //for (auto* node: this->open_list){
    //    if ((node->h_value + node->g_value) < (next_node->h_value + next_node->g_value)){
    //        next_node = node;
    //    }
    //}
    // // Remove each node from open_list
    //this->open_list.clear();

    // after mentor help
    std::sort(open_list.begin(), open_list.end(), CompareNodes);
    next_node = open_list.back();
    open_list.pop_back();

    // Return the node in open_list that has smallest f valude 
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    bool found_start = false;

    while (found_start == false){
        //auto parent_node = current_node->parent; // before mentor help
        //distance += parent_node->distance(*current_node); // before mentor help
        distance += current_node->parent->distance(*current_node); // after mentor help

        current_node = current_node->parent; // after mentor help

        //path_found.push_back(*parent_node); // before mentor help
        path_found.push_back(*current_node); // after mentor help
        
        //if (parent_node->index == this->start_node->index){ // index is private, so this cause an error...
        //if (parent_node->h_value == this->start_node->h_value){ // before mentor help
        if (current_node == this->start_node){ // after mentor help
            found_start = true;
        }
    }

    path_found.push_back(*start_node); // after mentor help

    reverse(begin(path_found), end(path_found));

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

// mentor help
// 1- mark start node as visited
// 2- add the start node to the open list
// 3-loop until the open list is empty
// 4-inside the loop you shall first pick the next node.
// 5-check if it is the intended node via checking against the end node
// 6-if yes then construct the final path then break
// 7-if no shall add another neighbor then iterate again.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    bool found_end = false;
    bool start_flag = true;
    RouteModel::Node *next_node = nullptr;
    
    while (found_end == false){
        // (1) set the current_node = start_node if it's the start of while loop
        if (start_flag == true){
            current_node = this->start_node;
            start_flag = false;
            // mentor help: 1- mark start node as visited
            current_node->visited = true;
            // mentor help: 2- add the start node to the open list
            this->open_list.push_back(current_node);
        }

        // (2) after the second loop
        // mentor help: 3-loop until the open list is empty
        while(this->open_list.empty() == false){
            // mentor help: 4-inside the loop you shall first pick the next node.
            //this->AddNeighbors(current_node); // before mentor help
            next_node = this->NextNode();

            // for the next loop, set the current_node
            current_node = next_node;

            // mentor help: 5-check if it is the intended node via checking against the end node
            // mentor help: 6-if yes then construct the final path then break
            // end process
            //if (next_node->h_value == this->end_node->h_value){ // before mentor help
            if (next_node == this->end_node){
                found_end = true;
                auto path_found = ConstructFinalPath(next_node);
                m_Model.path = path_found;
                break;
            }
            // mentor help: 7-if no shall add another neighbor then iterate again.
            else {
                this->AddNeighbors(current_node);
            }
        }
    }
}