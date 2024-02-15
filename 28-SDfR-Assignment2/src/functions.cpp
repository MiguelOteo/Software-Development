/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <array>
#include <string.h>
#include <windows.h> 
#include "../include/header.hpp"

/**
 * @brief Initializes the maze array with the maze pattern
 * 
 * Initializes a maze array with a default pattern using the following char legend:
 * "#" -> Walls
 * "." -> Paths
 * "x" -> Starting position
 * Then returns the array
 * 
 * @return array<array<char[2],12>,12> maze
*/
std::array<std::array<char[2], 12>, 12> initMaze()
{
    std::array<std::array<char[2], 12>, 12> maze =
    {{
        {"#", "#", "#", "#", "#", "#", "#", "#", "#", "#", "#", "#"},
        {"#", ".", ".", ".", "#", ".", ".", ".", ".", ".", ".", "#"},
        {".", ".", "#", ".", "#", ".", "#", "#", "#", "#", ".", "#"},
        {"#", "#", "#", ".", "#", ".", ".", ".", ".", "#", ".", "#"},
        {"#", ".", ".", ".", ".", "#", "#", "#", ".", "#", ".", "x"},
        {"#", "#", "#", "#", ".", "#", ".", "#", ".", "#", ".", "#"},
        {"#", ".", ".", "#", ".", "#", ".", "#", ".", "#", ".", "#"},
        {"#", "#", ".", "#", ".", "#", ".", "#", ".", "#", ".", "#"},
        {"#", ".", ".", ".", ".", ".", ".", ".", ".", "#", ".", "#"},
        {"#", "#", "#", "#", "#", "#", ".", "#", "#", "#", ".", "#"},
        {"#", ".", ".", ".", ".", ".", ".", ".", ".", ".", ".", "#"},
        {"#", "#", "#", "#", "#", "#", "#", "#", "#", "#", "#", "#"}
    }};
    return maze;
}

/**
 * @brief Initializes an auxiliary array of booleans all to false
 * 
 * Initializes a array of booleans equal in dimentions to the maze 
 * and sets it all to false. This array is used to know if the maze path 
 * tiles "." have already been gone through by the solving algorithm. 
 * 
 * @return array<array<bool,12>,12> visited
*/
std::array<std::array<bool, 12>, 12> initVisited()
{
    std::array<std::array<bool, 12>, 12> visited =
    {{
        false
    }};
    return visited;
}

/**
 * @brief Looks the entrance in the array of the maze 
 * 
 * Iterates over all the elements of the array of the maze until the 
 * entrance "x" is found, then returns the position in the array. 
 * If the entrance is not found the returned values is {-1, -1}
 * 
 * @param array<array<char[2],12>,12> maze
 * @return array<int,2> position
*/
std::pair<int, int> findMazeEntrance(std::array<std::array<char[2], 12>, 12> maze)
{
    std::pair<int, int> position = {-1, -1};

    for(int row = 0; row < 12; row++)
    {
        for(int column = 0; column < 12; column++)
        {
            if(strcmp(maze[row][column], "x") == 0)
            {
                position = {row, column};
                        return position;
            }
        }
    }
    return position;
} 

/**
 * @brief Looks the exit in the array of the maze 
 * 
 * Iterates over all the border elements of the array of the maze until the 
 * exit is found, then returns the position in the array. Exit is a valid tale "."
 * which is located at the border of the square array.
 * If the entrance is not found the returned values is {-1, -1}
 * 
 * Example boder elements:
 * 
 *  ##############
 *  #            #
 *  #            #
 *  .            #
 *  #            #
 *  ##############
 * 
 * @param array<array<char[2],12>,12> maze
 * @return array<int,2> position
*/
std::pair<int, int> findMazeExit(std::array<std::array<char[2], 12>, 12> maze)
{
    std::pair<int, int> position = {-1, -1};
    int row = 0;
    int column = 0;

    // Move allong all rows
    for(int row = 0; row < 12; row++)
    {
        // If the row is either the first or the last then iterate over all columns
        if(row == 0 || row == 11) 
        {
            for(int column = 0; column < 12; column++)
            {
                if(strcmp(maze[row][column],".") == 0)
                {
                    position = {row, column};
                    return position;
                }
            }
        }
        else // If not then only on the first and last element
        {
            if(strcmp(maze[row][0],".") == 0 || strcmp(maze[row][11],".") == 0)
            {
                    position = {row, column};
                    return position;
            }
        }
    } 
    return position;
} 

/**
 * @brief Prints the array in the console 
 * 
 * Iterates over all the elements of the array of the maze and prints it
 * in the console
 * 
 * @param array<array<char[2],12>,12> maze
*/
void printMaze(std::array<std::array<char[2], 12>, 12> maze)
{
    // Make each step of the algorithm visible for 0.2 seconds
    Sleep(200);

    // Cleans previous iteration of the algorithm
    system("CLS");

    for(int row = 0; row < 12; row++)
    {
        for(int column = 0; column < 12; column++)
        {
            std::cout << maze[row][column];
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// Enumerator of the directions in the array form the current position
enum Direction
{
    UP = -1,    // Move up by subtracting 1 to the row index
    DOWN = 1,   // Move down by adding 1 to the row index
    LEFT = -1,  // Move left by subtracting 1 to the column index
    RIGHT = 1   // Move right by adding 1 to the column index
};

/**
 * @brief Solves the maze using recursivity
 * 
 * Algorithm which solves the maze recursivetly by checking the posible 
 * movements form the current position:
 * 
 * - Valid movements from current position "x"
 * 
 *        .     Top:    Add -1 to the row of the current position
 *      . x .   Bottom: Add +1 to the row of the current position 
 *        .     Left:   Add -1 to the cxolumn of the current position
 *              Right:  Add +1 to the column of the current position
 * 
 *  It first marks the current postion as visited in the bool array, this will avoid
 *  going over the same tiles in a loop
 * 
 *  Then checks if the current position is at the boundary of the maze
 *  in that case it will ignore movements that would lead to getting out of the array index
 * 
 *  Then it gets the first next valid position, this is done by checking that the new position is
 *  neither a wall "#" nor a tile which has been visited previously. If both are true then it proceed
 *  to the new iteration.
 * 
 *  If none of the movements are valid the iteration will finish for that branch 
 *  of the recursive tree
 * 
 *  Possible solutions:
 *      The solution is reach when the current possiton is equal to the exit position (return true)
 *      If the last iteration returns false then no solution would have been found
 * 
 * @param array<array<char[2],12>,12> maze
 * @param array<array<bool,12>,12> visited
 * @param array<int,2> position
 * @param array<int,2> finish
 * @param int iteration
 * 
 * @return bool Solution found
*/
bool transverseMaze(std::array<std::array<char[2], 12>, 12> maze,
                    std::array<std::array<bool, 12>, 12> visited,
                    std::pair<int, int> currentPos,
                    std::pair<int, int> finish,
                    int iteration)
{   
    // Add the char "#" current tile to the maze and print it
    strcpy(maze[currentPos.first][currentPos.second], "x");
    printMaze(maze);

    // Checks of the current position is the exit, if it is exits the program
    if (currentPos.first == finish.first && currentPos.second == finish.second)
    {
        return true;
    }

    /*
       Next move around current position algrithm
    */

    // Array of all possible movements from a tile
    const std::array<std::pair<int, int>, 12> nextPositions = 
    {{
        {  UP,     0}, // Move UP    (Current position -1 on the row)
        {DOWN,     0}, // Move DOWN  (Current position 1 on the row)
        {   0, LEFT }, // Move LEFT  (Current position -1 on the column)
        {   0, RIGHT}  // Move RIGHT (Current position 1 on the column)
    }};

    // Sets the current position as visited
    visited[currentPos.first][currentPos.second] = true;

    // Iterate over all the possible possitions
    for (const auto& nextPos: nextPositions) 
    {
        // Get the indexes of the nextPosition
        int nextRow = currentPos.first + nextPos.first;
        int nextColumn = currentPos.second + nextPos.second;

        // Check that the new indexes are within the array range
        if(nextRow < 12 && nextRow >= 0 && nextColumn < 12 && nextColumn >= 0)
        {
            // Get the next tile char
            char* nextTile = maze[nextRow][nextColumn];

            // Verify that the tile is not a wall and that the tile has not been visited
            if (strcmp(nextTile, "#") != 0 && visited[nextRow][nextColumn] == false)
            {
                if(transverseMaze(maze, visited, {nextRow, nextColumn}, finish, iteration + 1))
                {
                    return true;
                };
            }
        }
    }
    return false;
}