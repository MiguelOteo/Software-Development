/**
 * @file Customer.cpp
 * @brief Implementation of the PrivateCustomer class, representing a customer of a delivery service.
 * 
 * This file defines the main constructor and accessor/mutator functions for 
 * the PrivateCustomer class. The class encapsulates details about the customer, 
 * adreess and nameof the customer.
 *  
 * @authors Miguel Oteo, Alvaro Redondo
*/

#include "../../include/obj/PrivateCustomer.hpp"

/**
 * @brief Main constructor for PrivateCustomer.
 * 
 * @param address Address of the Customer.
 * @param name Name of the PrivateCustomer.
 */
PrivateCustomer::PrivateCustomer(Address address, Name name) : 
    Customer(address), name(name) {}

/**
 *  Getters and Setters
 */

/**
 * @brief Get the Name of the private customer.
 * 
 * @return The name of the private customer.
 */
Name PrivateCustomer::getName()
{
    return this->name;
}

/**
 * @brief Set the name of the private customer.
 * 
 * @param name The new name of the private customer.
 */
void PrivateCustomer::setName(Name name)
{
    this->name = name;
}

/**
 *  Methods
 */

/**
 * @brief Turns the customer name to a list of strings to print it.
 * 
 * Iterates over the customer name of the customer and returns it as a list 
 * of the customer name private customer
 * 
 * @return List with all the customer name of the private customer.
 */
std::list<std::string>& PrivateCustomer::toStringName
    (std::list<std::string>& customerData)
{
    customerData.push_back("\nName\n");
    customerData.push_back("First Name: " + name.name);
    customerData.push_back("Middle Name: " + name.middleName);
    customerData.push_back("Last Name: " + name.lastName);

    return customerData;
}

/**
 * @brief Turns the object data to a list of strings to print it.
 * 
 * Iterates over the information of the customer and returns it as a list 
 * of all the data of the private customer
 * 
 * @return List with all the data of the private customer.
 */
std::list<std::string> PrivateCustomer::toString() 
{
    std::list<std::string> customerData;

    Address address = this->getAddress();
    Name name = this->name;

    customerData = toStringAddress(customerData);

    customerData = toStringName(customerData);

    return customerData;
}