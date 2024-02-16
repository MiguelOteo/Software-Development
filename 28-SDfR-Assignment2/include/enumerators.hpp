/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#ifndef ENUMERATORS_H 
#define ENUMERATORS_H

// Enumerator of the results from looking at the entrance and exit of the maze
enum InOutFound
{
    NOT_FOUND = -1,
    FOUND = 0
};

// Enumerator of the outcomes of searching for the exit of the maze doring the iteration
enum SolutionFound
{
    SOLUTION_NOT_FOUND = false,
    SOLUTION_FOUND = true
};

// Enumerator of the directions in the array form the current position
enum Direction
{
    UP = -1,    // Move up by subtracting 1 to the row index
    DOWN = 1,   // Move down by adding 1 to the row index
    LEFT = -1,  // Move left by subtracting 1 to the column index
    RIGHT = 1   // Move right by adding 1 to the column index
};

#endif