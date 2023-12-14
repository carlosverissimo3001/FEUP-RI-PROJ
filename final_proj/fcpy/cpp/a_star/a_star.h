#pragma once

struct Node{

    //------------- BST parameters
    Node* left;
    Node* right;
    Node* up;

    //------------- A* parameters 
    Node* parent;
    float g;
    float f;


};

extern void astar(float params[], int params_size);
extern float final_path[2050];
extern int final_path_size;