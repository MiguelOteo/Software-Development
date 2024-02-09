/*
 *  Created At: 07-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <time.h>
#include <iostream>
using namespace std; // Standard namespace

/**
 * @class Human 
 * Defines human properties and methods
 */
class Human 
{
    private: // Parameters of the class
        string firstName; 
        string lastName;
        int day;
        int month;
        int year;

    public: // Class methods

        Human(string fName, string lName, int D, int M, int Y) // Default constructor
        {
            firstName = fName;
            lastName = lName;
            day = D;
            month = M;
            year = Y;
        }

        /**
         * Getters
         */
        string getFirstName() 
        {
            return firstName;
        }

        string getFullName() 
        {
            return firstName + " " + lastName;
        }

        std::string getBirthDate() 
        {
            return to_string(day) + "-" + to_string(month) + "-" + to_string(year);
        }

        /**
         * Setters
         */
        void setFirstName(string fName)
        {
            firstName = fName;
        }

        void setLastName(string lName) 
        {
            lastName = lName;
        }

        /**
         * Methods
         */

        /**
         * @brief Inserts the birth date of the patient
         * 
         * Checks if the day and month are within the correct logical range, 
         * days (From 1 to 31) and month (From 1 to 12), if the conditions are 
         * met the new date is stored if not the operation is ignored
         * 
         * @param D day of the new birth date 
         * @param M month of the new birth date 
         * @param Y year of the new birth date
         * @return void
         */
        void setBirthDate(int D, int M, int Y)
        {
            // Check if days and months are valid
            if(D > 0 && D < 31 && M > 0 && M < 13) 
            {
                day = D;
                month = M;
                year = Y;
            } 
        }

        /**
         * @brief Compuete the age in years
         * 
         * Checks the computer's current time and computes the difference between the 
         * birth date year and the current year to know the aproximate age in years.
         * 
         * @return int The age in years
         */
        int getAge() 
        {
            // Get the current time of the computer
            time_t currentTime = time(NULL);
            struct tm *timeNow = localtime(&currentTime);
            
            // Extract the year from it
            int currentYear = timeNow->tm_year + 1900;
            
            // Compute the difference
            return currentYear - year;
        }

        /**
         * @brief Compuete the maximum BMPs of the patient
         * 
         * Computes the maximum BPMs of the patient by substracting the current age
         * of the patient in year to the value of 220
         * 
         * @return int The maximum BPMs of the patient
         */
        int calculateMaximumHeartRate() 
        {
            const int maxRate = 220;
            return maxRate - getAge();
        }

        /**
         * @brief Computes the target BPMs
         * 
         * Computes the target BPMs based on the maximum BPM of the patient, then returns 
         * the minimum and maximum values of the range
         * 
         * @return std::pair The min and max value of the target BPMs
         */
        pair<float, float> calculateTargetHeartRates() 
        {
            int maxRate = calculateMaximumHeartRate();
            return make_pair(maxRate*0.5, maxRate*0.85);
        }
};  