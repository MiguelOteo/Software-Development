/**
 * @file TwoDayPackage.cpp
 * @brief Implementation of the TwoDayPackage class, representing a two-day package derived from the Package class.
 * 
 * This file defines the default constructor and functionality for calculating the cost of a two-day package
 * based on its weight. The class extends the basic Package class and includes specific functionality
 * for calculating the cost of a two-day package.
 * 
 * @authors Miguel Oteo, Alvaro Redondo 
 */

#include <cmath>
#include <iostream>
#include "../../include/obj/Package.hpp"
#include "../../include/obj/TwoDayPackage.hpp"

/**
 * @brief Main constructor for TwoDayPackage.
 * 
 * @param weight The weight of the package.
 */
TwoDayPackage::TwoDayPackage(float weight) : Package(weight) {}

/**
 * @brief Calculate the cost of a two-day package based on its weight.
 * 
 * This function computes the cost of a two-day package using a formula that
 * includes a base cost and weight-dependent charges.
 * 
 * @return The total cost of the two-day package.
 */
float TwoDayPackage::calculateCost() 
{
    float cost = 0;
            
    // Cost of two-day packages
    return 5 + this->getWeight() * 2.5;
}

