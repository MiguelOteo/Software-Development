/*
 *  Created At: 12-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <array>

int main ( ) 
{
    // Create a std::array
    std::array<int, 5> a = {1, 2, 3, 4, 5};

    // Loop through the values using a range−based loop
    // and au to keyword
    for ( const auto& element : a ) 
    {
        std::cout << element << " " ;
    }

    std::cout << std::endl;
    
    return 0;
}