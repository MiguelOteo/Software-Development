/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <array>

#ifndef HEADER_H 
#define HEADER_H

std::array<std::array<char[2], 12>, 12> initMaze();

void printMaze(std::array<std::array<char[2], 12>, 12>);

std::array<int, 2> findMazeEntrance(std::array<std::array<char[2], 12>, 12>);

std::array<std::array<bool, 12>, 12> initVisited();

void transverseMaze(std::array<std::array<char[2], 12>, 12>,
                    std::array<std::array<bool, 12>, 12>,
                    std::array<int, 2>,
                    std::array<int, 2>, 
                    int);

#endif
