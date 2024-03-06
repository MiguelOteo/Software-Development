/**
 * @file Customer.hpp
 * @brief Header file for the Customer class.
 * 
 * This file defines the Customer class. The Customer class is designed to 
 * facilitate the management of customer data within a larger software system.
 * It includes a main constructor for initializing a customer with an address, 
 * as well as getter and setter methods for accessing and modifying the 
 * customer's address.
 * 
 * @author Miguel Oteo, Alvaro Redondo
*/

#ifndef COSTUMER_HPP
#define COSTUMER_HPP

#include <iostream>
#include <list>
#include "../str/structs.hpp"

class Customer
{
    private:
        Address address;  ///< Address of the customer.

    public:
    
        /**
         * @brief Main constructor for Customer.
         * 
         * @param address Address of the customer.
         */
        Customer(Address address);

        /**
         *  Getters and Setters
         */

        /**
         * @brief Get the address of the customer.
         * 
         * @return The address of the customer.
         */
        Address getAddress();

        /**
         * @brief Set the address of the customer.
         * 
         * @param address The new address of the customer.
         */
        void setAddress(Address address);

        /**
         *  Methods
         */

        /**
         * @brief Turns the address to a list of strings to print it.
         * 
         * Iterates over the address of the customer and returns it as a list 
         * of the address of the customer
         * 
         * @return List with the address of the business customer.
         */
        std::list<std::string>& toStringAddress
            (std::list<std::string>& customerData);

        /**
         * @brief Virtual function to return the data of the customer.
         * 
         * This function should be overridden by derived classes.
         * 
         * @return The data of the customer.
         */
        virtual std::list<std::string> toString() = 0;
};

#endif // COSTUMER_HPP