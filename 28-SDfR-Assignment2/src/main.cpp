/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <array>
#include "../include/header.hpp"

int main() // Initializetion of the main loop
{
    // Init of the maze 
    std::array<std::array<char[2], 12>, 12> maze = initMaze();

    // Init of the visited matrix
    std::array<std::array<bool, 12>, 12> visited = initVisited();

    // Search for the entrance
    std::array<int, 2> start = findMazeEntrance(maze);

    // Setup of the exit
    std::array<int, 2> finish = findMazeExit(maze);

    int iteration = 0;

    transverseMaze(maze, visited, start, finish, iteration);
}