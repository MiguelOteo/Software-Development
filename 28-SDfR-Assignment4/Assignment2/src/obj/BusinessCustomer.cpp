/**
 * @file BusinessCustomer.cpp
 * @brief Implementation of the BusinessCustomer class, representing a company customer of a delivery service.
 * 
 * This file defines the main constructor and accessor/mutator functions for 
 * the BusinessCustomer class. The class encapsulates details about the customer, 
 * adreess of the customer, the data of the company and a contact person name.
 *  
 * @authors Miguel Oteo, Alvaro Redondo
*/

#include "../../include/obj/BusinessCustomer.hpp"

/**
 * @brief Main constructor for BusinessCustomer.
 * 
 * @param address Address of the Customer.
 * @param companyData Data of the company of the BusinessCustomer.
 * @param contactName Name of the BusinessCustomer.
 */
BusinessCustomer::BusinessCustomer(Address address, CompanyData companyData, Name contactName) :
    Customer(address), companyData(companyData), contactName(contactName) {}

/**
 *  Getters and Setters
 */

/**
 * @brief Get the company data of the business customer.
 * 
 * @return The company data of the business customer.
 */
CompanyData BusinessCustomer::getCompanyData()
{
    return this->companyData;
}

/**
 * @brief Set the company data of the business customer.
 * 
 * @param name The new company data of the business customer.
 */
void BusinessCustomer::setCompanyData(CompanyData companyData)
{
    this->companyData = companyData;
}

/**
 * @brief Get the contact name of the business customer.
 * 
 * @return The contact name of the business customer.
 */
Name BusinessCustomer::getContactName()
{
    return this->contactName;
}

/**
 * @brief Set the contact name of the business customer.
 * 
 * @param contactName The new name of the business customer.
 */
void BusinessCustomer::setContactName(Name contactName)
{
    this->contactName = contactName;
}

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
std::list<std::string>& BusinessCustomer::toStringContactPerson
    (std::list<std::string>& customerData)
{
    customerData.push_back("\nContact Name\n");
    customerData.push_back("First Name: " + contactName.name);
    customerData.push_back("Middle Name: " + contactName.middleName);
    customerData.push_back("Last Name: " + contactName.lastName);

    return customerData;
}

/**
 * @brief Turns the company data to a list of strings to print it.
 * 
 * Iterates over the company data of the customer and returns it as a list 
 * of the company data business customer
 * 
 * @return List with all the company data of the business customer.
 */
std::list<std::string>& BusinessCustomer::toStringCompanyData
    (std::list<std::string>& customerData)
{
    customerData.push_back("\nCompany Data\n");
    customerData.push_back("Company Name: " + companyData.name);
    customerData.push_back("CID: " + companyData.CID);

    return customerData;
}

/**
 * @brief Turns the object data to a list of strings to print it.
 * 
 * Iterates over the information of the customer and returns it as a list 
 * of all the data of the business customer
 * 
 * @return List with all the data of the business customer.
 */
std::list<std::string> BusinessCustomer::toString() 
{
    std::list<std::string> customerData;

    Address address = this->getAddress();
    CompanyData companyData = this->companyData;
    Name contactName = this->contactName;

    customerData = toStringAddress(customerData);

    customerData = toStringCompanyData(customerData);

    customerData = toStringContactPerson(customerData);

    return customerData;
}