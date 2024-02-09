/*
 *  Created At: 07-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include "human.cpp"
#include <iostream>
using namespace std; // Standard namespace

int main() { // Starts main loop

    // Initialization of variables
    string fName;
    string lName;
    string day;
    string month;
    string year;

    // Stores the patient's name
    cout << "Insert the patient's first name: ";
    cin >> fName;

    // Stores the patient's last name
    cout << "Insert the patient's last name: ";
    cin >> lName;

    // Stores the patient's birth date
    cout << "Insert the patient's birth date (DD MM YYYY): ";
    cin >> day >> month >> year;

    // Creates a new object of type human using the constructor
    Human human = Human(fName, lName, 9, 2, 1999);

    // Print all the information of the patient in the console 
    cout << "Full name: " << human.getFullName() << endl;
    cout << "Birth Date: " << human.getBirthDate() << endl;
    cout << "Age: " << human.getAge() << endl;
    cout << "Max. Heart Rate: " << human.calculateMaximumHeartRate() << endl;
    pair<float, float> tRate = human.calculateTargetHeartRates();
    cout << "Target Heart Rate: " << tRate.first << "-" << tRate.second << endl;
}