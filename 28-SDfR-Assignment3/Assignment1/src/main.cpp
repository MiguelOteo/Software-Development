//==============================================================
// Filename    :  main.cpp
// Authors     :  Miguel Oteo, Álvaro Redondo
// Group       :  23
// Description :  Initialization of the main class
//==============================================================

#include "../include/List.h"
#include "../include/functions.hpp"

int main() // Init of the main loop
{
    // Initialization of variables
    List<char> firstList;
    List<char> secondList;
    List<char> thirdList;
    List<char> fourthList;

    // Filling the data into the lists
    fillList(firstList, "singlylinkedlist");
    fillList(secondList, "abcdefg");
    fillList(thirdList, "hijklmnop");
    fillList(fourthList, "qrstuvw");

    // Print the list with the insreted data
    firstList.print();
    secondList.print();

    // Concatenate the lists 
    secondList.concatenate(thirdList);
    secondList.concatenate(fourthList);

    // Print the list after the concatenation
    secondList.print();

    return 0;
}
