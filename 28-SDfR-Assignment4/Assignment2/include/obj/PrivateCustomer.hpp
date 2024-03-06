/**
 * @file PrivateCustomer.hpp
 * @brief Header file for the PrivateCustomer class.
 * 
 * This file defines the PrivateCustomer class, which is derived from 
 * the base Customer class. PrivateCustomer extends the basic customer 
 * information by including a Name attribute. This class represents a 
 * customer with private (individual) details, such as their name in 
 * addition to their address.
 * 
 * The PrivateCustomer class includes a main constructor for 
 * initializing a private customer with both an address and a name. 
 * It also provides getter and setter methods for accessing and modifying 
 * the private customer's name.
 * 
 * @author Miguel Oteo, Alvaro Redondo
*/

#include <iostream>
#include <list>
#include "Costumer.hpp"
#include "../str/structs.hpp"

class PrivateCustomer: public Customer
{
    private:
        Name name;  ///< Name of the private customer.

    public:
    
        /**
         * @brief Main constructor for PrivateCustomer.
         * 
         * @param address Address of the Customer.
         * @param name Name of the PrivateCustomer.
         */
        PrivateCustomer(Address address, Name name);

        /**
         *  Getters and Setters
         */

        /**
         * @brief Get the Name of the private customer.
         * 
         * @return The name of the private customer.
         */
        Name getName();

        /**
         * @brief Set the name of the private customer.
         * 
         * @param name The new name of the private customer.
         */
        void setName(Name name);

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
        std::list<std::string>& toStringName
            (std::list<std::string>& customerData);

        /**
         * @brief Turns the object data to a list of strings to print it.
         * 
         * Iterates over the information of the customer and returns it as a list 
         * of all the data of the private customer
         * 
         * @return List with all the data of the private customer.
         */
        std::list<std::string> toString() override;
};
