/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <array>
#include "../include/header.hpp"

int main() // Initializetion of the main loop
{
    std::array<std::array<char[2], 12>, 12> maze = initMaze();
    std::array<std::array<bool, 12>, 12> visited = initVisited();
    std::array<int, 2> start = findMazeEntrance(maze);
    std::array<int, 2> finish = {2, 0};
    int iteration = 0;

    transverseMaze(maze, visited, start, finish, iteration);
}