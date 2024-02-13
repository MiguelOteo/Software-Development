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
std::array<int, 2> findMazeEntrance(std::array<std::array<char[2], 12>, 12> maze)
{
    std::array<int, 2> position = {-1, -1};

    for(int row = 0; row < 12; row++)
    {
        for(int column = 0; column < 12; column++)
        {
            if(strcmp(maze[row][column],"x") == 0)
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
 *        .     Left:   Add -1 to the column of the current position
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
 *  Solution:
 *      The solution is reach when the current possiton is equal to the exit position
 *      
 *  Failing case:
 *      If the algorithm gets stuck it will stop after 100 iterations
 * 
 * @param array<array<char[2],12>,12> maze
 * @param array<array<bool,12>,12> visited
 * @param array<int,2> position
 * @param array<int,2> finish
 * @param int iteration
*/
void transverseMaze(std::array<std::array<char[2], 12>, 12> maze,
                    std::array<std::array<bool, 12>, 12> visited,
                    std::array<int, 2> position,
                    std::array<int, 2> finish,
                    int iteration)
{
    // Break point if iteration reaches 100
    if (iteration == 100)
    {
        std::cout << "Solution could not be found" << std::endl;
        exit(-1);
    }

    // Checks of the current position is the exit, if it is exits the program
    if (position[0] == finish[0] && position[1] == finish[1])
    {
        strcpy(maze[position[0]][position[1]], "x");
        printMaze(maze);
        std::cout << "Solution found" << std::endl;
        exit(0);
    }

    // Sets the current position as visited
    visited[position[0]][position[1]] = true;

    /*
        Check possible positions around the current positions
    */ 

    // Position on top is current position -1 on the row
    if (position[0] > 0)
    {
        // Get new position
        char* top = maze[position[0] - 1][position[1]];

        // Checks that the new possition is not a wall neither it has been visited before
        if (strcmp(top, "#") != 0 && visited[position[0] - 1][position[1]] == false)
        {
            std::array<int, 2> nextPosition = {position[0] - 1, position[1]};
            strcpy(maze[position[0]][position[1]], "x");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the bottom is current position +1 on the row
    if (position[0] < 11)
    {
        // Get new position
        char* bottom = maze[position[0] + 1][position[1]];

        // Checks that the new possition is not a wall neither it has been visited before
        if (strcmp(bottom, "#") != 0 && visited[position[0] + 1][position[1]] == false)
        {
            std::array<int, 2> nextPosition = {position[0] + 1, position[1]};
            strcpy(maze[position[0]][position[1]], "x");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the left is current position -1 on the columns
    if (position[1] > 0)
    {
        // Get new position
        char *left = maze[position[0]][position[1] - 1];

        // Checks that the new possition is not a wall neither it has been visited before
        if (strcmp(left, "#") != 0 && visited[position[0]][position[1] - 1] == false)
        {
            std::array<int, 2> nextPosition = {position[0], position[1] - 1};
            strcpy(maze[position[0]][position[1]], "x");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the right is current position +1 on the columns
    if (position[1] < 11)
    {
        // Get new position
        char *right = maze[position[0]][position[1] + 1];

        if (strcmp(right, "#") != 0 && visited[position[0]][position[1] + 1] == false)
        {
            std::array<int, 2> nextPosition = {position[0], position[1] + 1};
            strcpy(maze[position[0]][position[1]], "x");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }
}