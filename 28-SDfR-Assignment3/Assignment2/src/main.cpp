//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

#include <iostream>
#include "../include/Tree.h"
#include "../include/TreeNode.h"

int main() // Init of main loop
{   
    // Initialization of variables
    Tree<int> tree; 
    int intArray[10] = {4,7,2,8,3,9,0,1,5,6};

    for(int datum: intArray) // Iterate the array to to 
    {
        tree.insertNode(datum); // Insert the a new node to the tree
    }

    // Get the value from the user
    int valueToSearch = -1;
    std::cout << "Insert the value to search in the tree: ";
    std::cin >> valueToSearch;

    TreeNode<int>* ptr = tree.search(valueToSearch); // Searches the wanted value

    if(ptr == nullptr) // If found prints pointer memory location
    {
        std::cout << "Value " << valueToSearch << " could not be found in tree" << std::endl;
    }
    else // If not found print a message
    {
        std::cout << "Value " << valueToSearch << " found in tree position " << ptr << std::endl;
    }

    tree.outputTree(); // Print the tree 

    return 0;
}
