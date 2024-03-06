#include <iostream>
#include <list>
#include "../../include/obj/package.hpp"

/**
 * @brief Print the cost of each package in the given list.
 * 
 * This function iterates through a list of Package pointers and calls the calculateCost
 * function for each package, printing the cost.
 * 
 * @param packages The list of Package pointers.
 */
void printCost(std::list<Package*> packages)
{
    for (auto* package : packages)
    {
        // Call calculateCost for each package and print the result
        float cost = package->calculateCost();
        std::cout << "Package cost: " << cost << std::endl;
    }
}