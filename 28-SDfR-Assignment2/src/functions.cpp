#include <iostream>
#include <array>
#include <string.h>
#include <windows.h> 
#include "../include/header.hpp"

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

std::array<std::array<bool, 12>, 12> initVisited()
{
    std::array<std::array<bool, 12>, 12> visited =
    {{
        false
    }};
    return visited;
}

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

void printMaze(std::array<std::array<char[2], 12>, 12> maze)
{
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

void printVisited(std::array<std::array<bool, 12>, 12> maze)
{
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

void transverseMaze(std::array<std::array<char[2], 12>, 12> maze,
                    std::array<std::array<bool, 12>, 12> visited,
                    std::array<int, 2> position,
                    std::array<int, 2> finish,
                    int iteration)
{
    Sleep(300);

    if (iteration == 100)
    {
        std::cout << "Solution could not be found" << std::endl;
        exit(-1);
    }

    if (position[0] == finish[0] && position[1] == finish[1])
    {
        strcpy(maze[position[0]][position[1]], "x");
        system("CLS");
        printMaze(maze);
        std::cout << "Solution found" << std::endl;
        exit(0);
    }

    visited[position[0]][position[1]] = true;

    // Check positions around the current point

    // Position on top is current position -1 on the row
    if (position[0] > 0)
    {
        char* top = maze[position[0] - 1][position[1]];

        if (strcmp(top, "#") != 0 && visited[position[0] - 1][position[1]] == false)
        {
            std::array<int, 2> nextPosition = {position[0] - 1, position[1]};
            strcpy(maze[position[0]][position[1]], "x");
            system("CLS");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the bottom is current position +1 on the row
    if (position[0] < 11)
    {
        char* bottom = maze[position[0] + 1][position[1]];

        if (strcmp(bottom, "#") != 0 && visited[position[0] + 1][position[1]] == false)
        {
            std::array<int, 2> nextPosition = {position[0] + 1, position[1]};
            strcpy(maze[position[0]][position[1]], "x");
            system("CLS");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the left is current position -1 on the columns
    if (position[1] > 0)
    {
        char *left = maze[position[0]][position[1] - 1];

        if (strcmp(left, "#") != 0 && visited[position[0]][position[1] - 1] == false)
        {
            std::array<int, 2> nextPosition = {position[0], position[1] - 1};
            strcpy(maze[position[0]][position[1]], "x");
            system("CLS");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    // Position on the right is current position +1 on the columns
    if (position[1] < 11)
    {
        char *right = maze[position[0]][position[1] + 1];

        if (strcmp(right, "#") != 0 && visited[position[0]][position[1] + 1] == false)
        {
            std::array<int, 2> nextPosition = {position[0], position[1] + 1};
            strcpy(maze[position[0]][position[1]], "x");
            system("CLS");
            printMaze(maze);
            transverseMaze(maze, visited, nextPosition, finish, iteration + 1);
        }
    }

    visited[position[0]][position[1]] = false;
}