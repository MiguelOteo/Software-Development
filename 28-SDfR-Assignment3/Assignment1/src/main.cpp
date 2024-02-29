//==============================================================
// Filename    :  main.cpp
// Authors     :  Miguel Oteo, Álvaro Redondo
// Group       :  23
// License     :  N.A. or opensource license like LGPL
// Description :  Initialization of the main class
//==============================================================

#include "../include/List.h"
#include "../include/functions.hpp"

int main() // Init of the main loop
{
    List<char> firstList;
    List<char> secondList;
    List<char> thirdList;
    List<char> fourthList;

    firstList = fillList(firstList, "singlylinkedlist");
    secondList = fillList(secondList, "abcdefg");

    firstList.print();
    secondList.print();

    thirdList = fillList(thirdList, "hijklmnop");
    fourthList = fillList(fourthList, "qrstuvw");

    secondList.concatenate(thirdList);
    secondList.concatenate(fourthList);

    secondList.print();

    return 0;
}
