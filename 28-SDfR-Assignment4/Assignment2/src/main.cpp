#include <iostream>
#include <list>
#include "../include/console/printFunctions.hpp"
#include "../include/obj/TwoDayPackage.hpp"
#include "../include/obj/OverNightPackage.hpp"
#include "../include/obj/BusinessCustomer.hpp"
#include "../include/obj/PrivateCustomer.hpp"
#include "../include/str/structs.hpp"

int main() { // Init of the main loop

    // Create a list to store the packages
    std::list<Package*> packages;
    std::list<Customer*> customers;

    float weight = 1.4;
    float weight2 = 1.4;
    float weight3 = 3.4;

    Address addressTest = 
    {
        "434",       // Number
        "Gran Via",  // Street
        "Madrid",    // City
        "Spain",     // Country
        "28013",     // Postal code
    };

    Name nameTest = 
    {
        "Miguel",  // Name
        "",        // Middle name
        "Oteo"     // Last name
    };

    CompanyData companyDataTest =
    {
        "Twente University",  // Company name
        "7346283LGHD"         // CID
    };

    BusinessCustomer* bussCustomerEmpty = new BusinessCustomer(addressTest, companyDataTest, nameTest);
    PrivateCustomer* privCustomerEmpty = new PrivateCustomer(addressTest, nameTest);
    BusinessCustomer* bussCustomer = new BusinessCustomer(addressTest, companyDataTest, nameTest);
    BusinessCustomer* privCustomer = new BusinessCustomer(addressTest, companyDataTest, nameTest);
    
    
    TwoDayPackage* package1 = new TwoDayPackage(weight, bussCustomer, privCustomer);
    OverNightPackage* package2 = new OverNightPackage(weight2, privCustomer, bussCustomer);
    OverNightPackage* package3 = new OverNightPackage(weight3, bussCustomer, privCustomer);
    
    customers.push_back(bussCustomer);
    customers.push_back(privCustomer);
    customers.push_back(privCustomerEmpty);
    customers.push_back(bussCustomerEmpty);

    packages.push_back(package1);
    packages.push_back(package2);
    packages.push_back(package3);

    for (auto& costumer: customers)
    {
        printCustomerInfo(costumer);
    }

    printChristmasCardContactPersons(customers);

    std::cout << "\n\nTest of function printCost()" << std::endl;
    printCost(packages);

    // Free allocated memory
    for (const auto& package : packages) {
        delete package;
    }

    return 0;
}
