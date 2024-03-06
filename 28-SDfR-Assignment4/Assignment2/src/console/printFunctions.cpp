#include <iostream>
#include <list>
#include "../../include/obj/package.hpp"
#include "../../include/obj/BusinessCustomer.hpp"

/**
 * @brief Print the cost of each package in the given list.
 * 
 * This function iterates through a list of Package pointers and calls the calculateCost
 * function for each package, printing the cost.
 * 
 * @param packages The list of Package pointers.
 */
void printCost(std::list<Package*> packages)
{
    for (auto* package : packages)
    {
        // Call calculateCost for each package and print the result
        float cost = package->calculateCost();
        std::cout << "Package cost: " << cost << std::endl;
    }
}

/**
 * @brief Print customer information.
 * 
 * This function takes a Customer pointer and prints the information
 * returned by the toString method.
 * 
 * @param customer The Customer pointer.
 */
void printCustomerInfo(Customer* customer)
{
    // Retrieve customer information as a list of strings
    std::list<std::string> customerData = customer->toString();

    // Print each line of customer information
    for (auto& dataLine : customerData)
    {
        std::cout << dataLine << std::endl;
    }
}

/**
 * @brief Print contact persons for business customers on Christmas cards.
 * 
 * This function iterates through a list of Customer pointers, checks if each
 * customer is a BusinessCustomer, and if so, prints the contact person information
 * for Christmas cards using the toStringContactPerson method.
 * 
 * @param customers The list of Customer pointers.
 */
void printChristmasCardContactPersons(std::list<Customer*>& customers)
{
    // Iterate through the list of customers
    for (const auto& customer : customers) 
    {
        // Attempt to cast the customer pointer to a BusinessCustomer pointer
        const auto businessCustomer = dynamic_cast<BusinessCustomer*>(customer);

        // Skip non-business customers
        if (!businessCustomer) 
        {
            continue;
        }

        // Retrieve contact person information as a list of strings
        std::list<std::string> customerData;
        customerData = businessCustomer->toStringContactPerson(customerData);

        // Print each line of contact person information for Christmas cards
        for (auto& dataLine : customerData)
        {
            std::cout << dataLine << std::endl;
        }
    }
}
