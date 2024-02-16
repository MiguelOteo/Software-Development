/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <array>
#include "../include/interface.hpp"
#include "../include/enumerators.hpp"

int main() // Initializetion of the main loop
{
    // Init of the maze  
    std::array<std::array<char[2], 12>, 12> maze = initMaze();

    // Search for the entrance
    std::pair<int, int> start = findMazeEntrance(maze);

    // Setup of the exit
    std::pair<int, int> finish = findMazeExit(maze);

    // If entrance not found then exit
    if(start.first == NOT_FOUND || start.second == NOT_FOUND) 
    {
        std::cout << "No entrance to the maze was found" << std::endl;
        return NOT_FOUND;
    } 

    // If exit not found then exit
    if(finish.first == NOT_FOUND || finish.second == NOT_FOUND)
    {
        std::cout << "No exit to the maze was found" << std::endl;
        return NOT_FOUND;     
    }

    // Init of the visited matrix
    std::array<std::array<bool, 12>, 12> visited = initVisited();

    // Initialize the recursive algorithm
    if(transverseMaze(maze, visited, start, finish))
    {
        std::cout << "Solution found" << std::endl;
        return FOUND;
    };

    return NOT_FOUND;
}