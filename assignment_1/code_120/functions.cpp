/*
 *  Created At: 07-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include <cmath>
#include "header.h"
using namespace std; // Standard namespace

/**
 * @brief Computes the BMI and prints it
 *
 * Computes the BMI using the weight and height of the person and prints
 * the following results based on the BMI 
 * 
 * BMI TABLE
 * You are underweight:   less than 18.5
 * Your weight is normal: between 18.5 and 24.9
 * You are overweight:    between 25 and 29.9
 * You are obese:         30 or greater
 * 
 * @param float weight The weight of the person in [Kg]
 * @param float height The h eight of the person in [m]
 * @return void
 */
void evaluateAndPrintBMI(const float weight, const float height)
{
    float BMI = weight/pow(height, 2);

    if(BMI < 18.5)
    {
        cout << "You are underweight";
        return;
    }
    if(BMI >= 18.5 && BMI <= 24.9) 
    {
        cout << "Your weight is normal";
        return;
    }
    if(BMI >= 25 && BMI <= 29.9) 
    {
        cout << "You are overweight";
        return;
    }
    if(BMI > 30)
    {
        cout << "You are obese";
        return;
    }

    cout << "BMI could not be defined";
}

/**
 * @brief Print BMI table
 * 
 * BMI TABLE
 * Underweight:  less than 18.5
 * Normal:       between 18.5 and 24.9
 * Overweight:   between 25 and 29.9
 * Obese:        30 or greater
 * 
 * @return void
 */
void printInfo() 
{
    cout << "BMI VALUES\n" + 
            "Underweight:  less than 18.5\n" + 
            "Normal:       between 18.5 and 24.9\n" +
            "Overweight:   between 25 and 29.9\n" +
            "Obese:        30 or greater\n\n";
}