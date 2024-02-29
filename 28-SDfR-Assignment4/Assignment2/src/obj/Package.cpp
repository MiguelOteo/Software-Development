/*
 *  Created At: 27-02-2024
 *  Authors: Miguel Oteo, Alvaro Redondo 
 */

#include <cmath>
#include <iostream>
#include "../../include/enums.hpp"
#include "../../include/structs.hpp"
#include "Customer.cpp"

class Package
{
    private: 
        TypePackage type;
        float weight;
        Customer* sender;
        Customer* receiver;

    public:

        // Constructor
        Package(TypePackage type, float weight, Customer* sender, Customer* receiver) : 
            type(type), weight(weight), sender(sender), receiver(receiver) {}

        /*
            Getters and Setters
        */

        // Type variable
        TypePackage getType() 
        {
            return this->type;
        }

        void setType(TypePackage type)
        {
            this->type = type;
        }

        // Weight variable
        float getWeight()
        {
            return this->weight;
        }

        void setWeight(float weight)
        {
            this->weight = weight;
        }

        // Sender variable
        Customer* getSender()
        {
            return this->sender;
        }

        void setSender(Customer* sender)
        {
            this->sender = sender;
        }

        // Receiver variable
        Customer* getReceiver()
        {
            return this->receiver;
        }

        void setReceiver(Customer* receiver)
        {
            this->receiver = receiver;
        }

        /*
            Methods
        */

        float calculateCost()
        {
            float cost = 0;
        
            // Base cost for all packages
            cost = 5 + weight*2.5;
           
            // Additional cost for OvernightPackage type
            if(this->type == OvernightPackage)
            {
                cost = cost + pow(weight, 2)*weight;
            }

            return cost;
        }

        void printCost()
        {
            float cost = calculateCost();

            std::cout << "Cost of the package is: " << cost << std::endl;
        }
};