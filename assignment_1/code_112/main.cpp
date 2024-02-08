/*
 *  Created At: 07-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include "header.h"
using namespace std; // Standard namespace

int main(void) // Starts main loop
{ 
    // Variable declarations
    int a = 7;
    int b = 3;

    // Compute the division
    int c = divide(a, b);

    // Print the result
    cout << a << "/" << b << "=" << c << endl;
    return 0;
}
