//==============================================================
// Filename    :  functions.hpp
// Authors     :  Miguel Oteo, Álvaro Redondo
// Group       :  23
// License     :  none
// Description :  Lib of the functions
//==============================================================

#include "../include/List.h"
#include <iostream>

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

/**
*  @brief Fills the list with the characters of the string 
*  
*  Iterates over all the elements of the string and appends them 
*  to the list
* 
*  @param List<char> list
*  @param std::string string
*  @return List<char>
*/
List<char> fillList(List<char> list, std::string string)
{
    for (char letter: string)
    {
        list.insertAtBack(letter);
    }

    return list;
}

#endif