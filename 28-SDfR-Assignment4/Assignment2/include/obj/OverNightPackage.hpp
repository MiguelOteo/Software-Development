/**
 * @file OverNightPackage.hpp
 * @brief Header file for the OverNightPackage class.
 * 
 * This file defines the OverNightPackage class, which is a derived class of the Package
 * class. It represents an overnight package with additional functionality for calculating
 * the cost based on weight, weight-dependent charges, and a weight squared term.
 * 
 * @author Miguel Oteo, Alvaro Redondo
*/

#ifndef OVERNIGHTPACKAGE_HPP
#define OVERNIGHTPACKAGE_HPP

#include "Package.hpp"

class OverNightPackage : public Package
{
    public:

        /**
         * @brief Main constructor for OverNightPackage.
         * 
         * @param weight The weight of the package.
         */
        OverNightPackage(float, Customer* sender, Customer* receiver);

        /**
         *  Methods
         */

        /**
         * @brief Calculate the cost of an overnight package based on its weight.
         * 
         * This function calculates the cost of an overnight package using a formula
         * that includes a base cost, weight-dependent charges, and a weight squared term.
         * 
         * @return The total cost of the overnight package.
         */
        float calculateCost() override;
};

#endif // OVERNIGHTPACKAGE_HPP
