/**
 * @file Customer.cpp
 * @brief Implementation of the Customer class, representing a customer of a delivery service.
 * 
 * This file defines the main constructor and accessor/mutator functions for 
 * the Customer class. The class encapsulates details about the customer, 
 * adreess of the customer.
 *  
 * @authors Miguel Oteo, Alvaro Redondo
*/

#include "../../include/obj/Costumer.hpp"

/**
 * @brief Main constructor for Customer.
 * 
 * @param address Address of the customer.
 */
Customer::Customer(Address address) : address(address) {}

/**
 *  Getters and Setters
 */

/**
 * @brief Get the address of the customer.
 * 
 * @return The address of the customer.
 */
Address Customer::getAddress()
{
    return this->address;
}

/**
 * @brief Set the address of the customer.
 * 
 * @param address The new address of the customer.
 */
void Customer::setAddress(Address address)
{
    this->address = address;
}

/**
 * @brief Turns the address to a list of strings to print it.
 * 
 * Iterates over the address of the customer and returns it as a list 
 * of the address of the customer
 * 
 * @return List with the address of the business customer.
 */
std::list<std::string>& Customer::toStringAddress
    (std::list<std::string>& customerData)
{
    customerData.push_back("\nPackage Adress\n");
    customerData.push_back("Street Number: " + address.streetNumber);
    customerData.push_back("Street: " + address.street);
    customerData.push_back("City: " + address.city);
    customerData.push_back("Country: " + address.country);
    customerData.push_back("Postal Code: " + address.postalCode);

    return customerData;
}
