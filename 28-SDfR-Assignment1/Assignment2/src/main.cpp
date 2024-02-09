/*
 *  Created At: 07-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include "../include/header.h"
using namespace std; // Standard namespace

int main() // Starts main loop
{
    // Declaration of variables
    float weight = 0;
    float height = 0;

    printInfo();

    cout << "Insert your height in m: ";
    cin >> height; // Store console input in variable height

    cout << "Insert your weight in Kg: ";
    cin >> weight; // Store console input in variable weight

    evaluateAndPrintBMI(weight, height);
    return 0;
}