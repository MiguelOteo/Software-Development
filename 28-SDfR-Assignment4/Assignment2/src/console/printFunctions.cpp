#include <iostream>
#include <list>
#include <algorithm>
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
void printCost(std::list<Package*>& packages)
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
void printChristmasCardContactPersons(std::list<Package*>& packages)
{
    // Maintain a list of unique business customers and their contact information
    std::list<Customer*> uniqueCustomers;
    std::list<std::string> customerData;

    // Iterate through the list of packages
    for (const auto& package : packages) 
    {
        // Get both sender and receiver from the package
        Customer* sender = package->getSender();
        Customer* receiver = package->getReceiver();

        // Attempt to cast the customer pointer to a BusinessCustomer pointer
        BusinessCustomer* senderBusinessCustomer = dynamic_cast<BusinessCustomer*>(sender);
        BusinessCustomer* receiverBusinessCustomer = dynamic_cast<BusinessCustomer*>(receiver);
        
        // Check and process sender if it is a business customer and has not been processed before
        if (senderBusinessCustomer != nullptr && 
            std::find(uniqueCustomers.begin(), uniqueCustomers.end(), senderBusinessCustomer) == uniqueCustomers.end())
        {
            // Retrieve contact person information as a list of strings
            customerData = senderBusinessCustomer->toStringContactPerson(customerData);
            uniqueCustomers.push_back(senderBusinessCustomer);
        }

        // Check and process receiver if it is a business customer and has not been processed before
        if (receiverBusinessCustomer != nullptr && 
            std::find(uniqueCustomers.begin(), uniqueCustomers.end(), receiverBusinessCustomer) == uniqueCustomers.end())
        {
            // Retrieve contact person information as a list of strings
            customerData = receiverBusinessCustomer->toStringContactPerson(customerData);
            uniqueCustomers.push_back(receiverBusinessCustomer);
        } 
    }

    // Print each line of contact person information
    for (auto& dataLine : customerData)
    {
        std::cout << dataLine << std::endl;
    }
}

