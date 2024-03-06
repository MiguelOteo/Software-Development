/**
 * @file BusinessCustomer.hpp
 * @brief Header file for the BusinessCustomer class.
 * 
 * This file defines the BusinessCustomer class, which is derived from 
 * the base Customer class. BusinessCustomer extends the basic customer 
 * information by including a Name and CompanyData attribute. This class 
 * represents a company customer details, such as their company data, 
 * a contact person name in addition to their address.
 * 
 * The BusinessCustomer class includes a main constructor for 
 * initializing a business customer with both an address a contact name and 
 * the company data. It also provides getter and setter methods for accessing 
 * and modifying the contact person's name and the company data.
 * 
 * @author Miguel Oteo, Alvaro Redondo
*/

#include <iostream>
#include <list>
#include "Costumer.hpp"
#include "../str/structs.hpp"

class BusinessCustomer: public Customer
{
    private:
        CompanyData companyData;  ///< Company data of the bussiness customer.
        Name contactName;         ///< Contact name of the bussiness customer.

    public:

        /**
         * @brief Main constructor for BusinessCustomer.
         * 
         * @param address Address of the Customer.
         * @param companyData Data of the company of the BusinessCustomer.
         * @param contactName Name of the BusinessCustomer.
         */
        BusinessCustomer(Address address, CompanyData companyData, Name contactName);

        /**
         *  Getters and Setters
         */

        /**
         * @brief Get the company data of the business customer.
         * 
         * @return The company data of the business customer.
         */
        CompanyData getCompanyData();

        /**
         * @brief Set the company data of the business customer.
         * 
         * @param name The new company data of the business customer.
         */
        void setCompanyData(CompanyData companyData);

        /**
         * @brief Get the contact name of the business customer.
         * 
         * @return The contact name of the business customer.
         */
        Name getContactName();

        /**
         * @brief Set the contact name of the business customer.
         * 
         * @param contactName The new name of the business customer.
         */
        void setContactName(Name contactName);

        /**
         *  Methods
         */

        /**
         * @brief Turns the contact data to a list of strings to print it.
         * 
         * Iterates over the contact data of the customer and returns it as a list 
         * of the contact data business customer
         * 
         * @return List with all the contact data of the business customer.
         */
        std::list<std::string>& toStringContactPerson
            (std::list<std::string>& customerData);

        /**
         * @brief Turns the company data to a list of strings to print it.
         * 
         * Iterates over the company data of the customer and returns it as a list 
         * of the company data business customer
         * 
         * @return List with all the company data of the business customer.
         */
        std::list<std::string>& toStringCompanyData
            (std::list<std::string>& customerData);

        /**
         * @brief Turns the object data to a list of strings to print it.
         * 
         * Iterates over the information of the customer and returns it as a list 
         * of all the data of the business customer
         * 
         * @return List with all the data of the business customer.
         */
        std::list<std::string> toString() override;
};