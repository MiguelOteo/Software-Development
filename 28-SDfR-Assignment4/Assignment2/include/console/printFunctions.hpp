#include <iostream>
#include <list>
#include "../obj/Package.hpp"
#include "../obj/Costumer.hpp"

#ifndef PRINTFUNCTIONS_H 
#define PRINTFUNCTIONS_H

/**
 * @brief Print the cost of each package in the given list.
 * 
 * This function iterates through a list of Package pointers and calls the calculateCost
 * function for each package, printing the cost.
 * 
 * @param packages The list of Package pointers.
 */
void printCost(std::list<Package*>& packages);

/**
 * @brief Print customer information.
 * 
 * This function takes a Customer pointer and prints the information
 * returned by the toString method.
 * 
 * @param customer The Customer pointer.
 */
void printCustomerInfo(Customer* customer);

/**
 * @brief Print contact persons for business customers once.
 * 
 * This function iterates through a list of Customer pointers, checks if each
 * customer is a BusinessCustomer, and if so, prints the contact person information
 * for Christmas cards using the toStringContactPerson method.
 * 
 * @param packages The list of Packages pointers.
 */
void printChristmasCardContactPersons(std::list<Package*>& packages);

#endif