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

    // Init of testing data
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

    Address addressTest2 = 
    {
        "324",     // Number
        "Murcia",  // Street
        "Ibiza",   // City
        "Spain",   // Country
        "23748",   // Postal code
    };

    Name nameTest = 
    {
        "Miguel",  // Name
        "",        // Middle name
        "Oteo"     // Last name
    };

    Name nameTest2 = 
    {
        "Alvaro",    // Name
        "",          // Middle name
        "Arredondo"  // Last name
    };

    CompanyData companyDataTest =
    {
        "Company 1",   // Company name
        "7346283LGHD"  // CID
    };

    CompanyData companyDataTest2 =
    {
        "Company 2",   // Company name
        "78472342GHE"  // CID
    };

    // Init of all test customers and packages
    BusinessCustomer* bussCustomerEmpty = new BusinessCustomer(addressTest, companyDataTest, nameTest2);
    BusinessCustomer* bussCustomer = new BusinessCustomer(addressTest2, companyDataTest, nameTest2);
    BusinessCustomer* bussCustomer2 = new BusinessCustomer(addressTest, companyDataTest2, nameTest);

    PrivateCustomer* privCustomerEmpty = new PrivateCustomer(addressTest, nameTest2);
    PrivateCustomer* privCustomer = new PrivateCustomer(addressTest2, nameTest);
    PrivateCustomer* privCustomer2 = new PrivateCustomer(addressTest2, nameTest2);

    TwoDayPackage* package1 = new TwoDayPackage(weight, bussCustomer, privCustomer2);
    OverNightPackage* package2 = new OverNightPackage(weight2, bussCustomer, bussCustomer2);
    OverNightPackage* package3 = new OverNightPackage(weight3, privCustomer, privCustomer);
    
    // Adding private customers to the list (random order)
    customers.push_back(bussCustomer);
    customers.push_back(bussCustomerEmpty);
    customers.push_back(privCustomer);
    customers.push_back(privCustomer2);
    customers.push_back(bussCustomer2);
    customers.push_back(privCustomerEmpty);
    
    packages.push_back(package1);
    packages.push_back(package2);
    packages.push_back(package3);

    std::cout << "\n\n\nTest of function printCustomerInfo()\n" << std::endl;
    // Print the customer data of each customer
    int count = 1;
    for (auto& costumer: customers)
    {
        std::cout << "\n\n################\nCUSTOMER " << count;
        printCustomerInfo(costumer);
        count = count + 1;
    }

    std::cout << "\n\n\nTest of function printChristmasCardContactPersons()\n" << std::endl;
    // Print the constact data of the customers which are bussiness customers
    printChristmasCardContactPersons(packages);

    std::cout << "\n\n\nTest of function printCost()\n" << std::endl;
    printCost(packages);

    // Free allocated memory
    for (const auto& package : packages) 
    {
        delete package;
    }

    for (const auto& customer : customers)
    {
        delete customer;
    }

    return 0;
}
