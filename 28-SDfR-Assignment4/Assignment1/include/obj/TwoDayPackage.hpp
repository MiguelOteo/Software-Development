/**
 * @file TwoDayPackage.hpp
 * @brief Header file for the TwoDayPackage class.
 * 
 * This file defines the TwoDayPackage class, which is a derived class of the Package
 * class. It represents a two-day package with additional functionality for calculating
 * the cost based on weight and other factors.
 * 
 * @authors Miguel Oteo, Alvaro Redondo
*/

#ifndef TWODAYPACKAGE_HPP
#define TWODAYPACKAGE_HPP

#include "Package.hpp"

class TwoDayPackage : public Package
{
    public:
        
        /**
         * @brief Main constructor for TwoDayPackage.
         * 
         * @param weight The weight of the package.
         */
        TwoDayPackage(float);

        /**
         *  Methods
         */

        /**
         * @brief Calculate the cost of a two-day package based on its weight.
         * 
         * This function computes the cost of a two-day package using a formula that
         * includes a base cost and weight-dependent charges.
         * 
         * @return The total cost of the two-day package.
         */
        float calculateCost() override;
};

#endif // TWODAYPACKAGE_HPP
