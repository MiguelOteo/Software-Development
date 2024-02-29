/*
 *  Created At: 27-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <iostream>

#ifndef STRUCTS_H 
#define STRUCTS_H

struct Address 
{
    int streetNumber;
    std::string street;
    std::string city;
    std::string country; 

    std::string postalCode;
};

struct Name
{
    std::string name;
    std::string middleName;
    std::string lastName;
};

struct CompanyData
{
    std::string name;
    std::string CID;
};

#endif