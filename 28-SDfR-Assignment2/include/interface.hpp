/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <array>

#ifndef INTERFACE_H 
#define INTERFACE_H

std::array<std::array<char[2], 12>, 12> initMaze();

void printMaze(std::array<std::array<char[2], 12>, 12>);

std::pair<int, int> findMazeEntrance(std::array<std::array<char[2], 12>, 12>);

std::pair<int, int> findMazeExit(std::array<std::array<char[2], 12>, 12>);

std::array<std::array<bool, 12>, 12> initVisited();

bool transverseMaze(std::array<std::array<char[2], 12>, 12>,
                    std::array<std::array<bool, 12>, 12>,
                    std::pair<int, int>,
                    std::pair<int, int>);

#endif
