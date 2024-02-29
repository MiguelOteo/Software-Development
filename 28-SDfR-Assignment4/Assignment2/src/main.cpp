/*
 *  Created At: 27-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include "obj/Customer.cpp"
#include "obj/Package.cpp"

int main() // Init of main loop
{
    Name name = Name();
    name.name = "Miguel";
    name.middleName = "";
    name.lastName = "Oteo";

    Address address = Address();
    address.city = "Madrid";
    address.country = "Spain";
    address.postalCode = "34242";
    address.street = "Gran Vía";
    address.streetNumber = 43;

   
}