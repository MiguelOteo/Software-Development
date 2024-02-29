/*
 *  Created At: 27-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>
#include "../../include/enums.hpp"
#include "../../include/structs.hpp"

class Customer
{
    private:
        Address address;

    public:
    
        // Constructor
        Customer(Address address) : address(address) {}

        /**
         * Getters and Setters
        */
        Address getAddress()
        {
            return this->address;
        }

        void setAddress(Address address)
        {
            this->address = address;
        }
};

class PrivateCustomer: public Customer
{
    private:
        Name name;

    public:
    
        // Constructor
        PrivateCustomer(Address address, Name name) : 
            Customer(address), name(name) {}

        /**
         * Getters and Setters
        */
        Name getName()
        {
            return this->name;
        }

        void setName(Name name)
        {
            this->name = name;
        }

};

class BusinessCustomer: public Customer
{
    private:
        CompanyData companyData;
        Name contactName;

    public:

        // Constructor
        BusinessCustomer(Address address, CompanyData companyData, Name contactName) :
            Customer(address), companyData(companyData), contactName(contactName) {}

        /**
         * Getters and Setters
        */
        CompanyData getCompanyData()
        {
            return this->companyData;
        }

        void setCompanyData(CompanyData companyData)
        {
            this->companyData = companyData;
        }

        Name getContactName()
        {
            return this->contactName;
        }

        void setContactName(Name contactName)
        {
            this->contactName = contactName;
        }
};