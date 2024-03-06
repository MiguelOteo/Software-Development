/**
 * @file OverNightPackage.cpp
 * @brief Implementation of the OverNightPackage class, representing an overnight package derived from the Package class.
 * 
 * This file defines the main constructor and functionality for calculating the cost of an overnight package
 * based on its weight. The class extends the basic Package class and includes specific functionality
 * for calculating the cost of an overnight package.
 * 
 * @authors Miguel Oteo, Alvaro Redondo 
 */

#include <cmath>
#include <iostream>
#include "../../include/obj/Package.hpp"
#include "../../include/obj/OverNightPackage.hpp"

/**
 * @brief Main constructor for OverNightPackage.
 * 
 * @param weight The weight of the package.
 */
OverNightPackage::OverNightPackage(float weight) : Package(weight) {}

/**
 * @brief Calculate the cost of an overnight package based on its weight.
 * 
 * This function calculates the cost of an overnight package using a formula
 * that includes a base cost, weight-dependent charges, and a weight squared term.
 * 
 * @return The total cost of the overnight package.
 */
float OverNightPackage::calculateCost()
{
    float cost = 0;
            
    // Cost of overnight packages
    return 5 + this->getWeight() * 2.5 + pow(this->getWeight(), 2) * 1.10;
}

