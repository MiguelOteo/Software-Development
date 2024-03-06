#include <iostream>
#include <list>
#include "../include/console/printFunctions.hpp"
#include "../include/obj/TwoDayPackage.hpp"
#include "../include/obj/OverNightPackage.hpp"
#include "../include/str/structs.hpp"

int main() { // Init of the main loop

    // Create a list to store the packages
    std::list<Package*> packages;

    float weight = 1.4;
    float weight2 = 1.4;
    float weight3 = 3.4;

    Address addressTest = 
    {
        434,         // Number
        "Gran Via",  // Street
        "Madrid",    // City
        "Spain",     // Country
        "28013",     // Postal code
    };

    Name nameTest = 
    {
        "Miguel",
        "",
        "Oteo"
    };
    
    TwoDayPackage* package1 = new TwoDayPackage(weight);
    package1->setAddressReceiver(addressTest);
    package1->setAddressSender(addressTest);
    package1->setReceiverName(nameTest);
    package1->setSenderName(nameTest);
    OverNightPackage* package2 = new OverNightPackage(weight2);
    OverNightPackage* package3 = new OverNightPackage(weight3);
    
    packages.push_back(package1);
    packages.push_back(package2);
    packages.push_back(package3);

    std::cout << "\nTest of the methods" << std::endl;
    std::cout << "Struct Address Receiver" << std::endl;
    std::cout << package1->getAddressReceiver().streetNumber << std::endl;
    std::cout << package1->getAddressReceiver().street << std::endl;
    std::cout << package1->getAddressReceiver().city << std::endl;
    std::cout << package1->getAddressReceiver().country << std::endl;
    std::cout << package1->getAddressReceiver().postalCode << std::endl;

    std::cout << "\nStruct Name Sender" << std::endl;
    std::cout << package1->getSenderName().name << std::endl;
    std::cout << package1->getSenderName().middleName << std::endl;
    std::cout << package1->getSenderName().lastName << std::endl;

    std::cout << "\n\nTest of function printCost()" << std::endl;
    printCost(packages);

    // Free allocated memory
    for (const auto& package : packages) {
        delete package;
    }

    return 0;
}
