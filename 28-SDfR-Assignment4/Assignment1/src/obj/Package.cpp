#include <cmath>
#include <iostream>
#include "../../include/enums.hpp"
#include "../../include/structs.hpp"

class Package
{
    private: 
        TypePackage type;
        float weight;

        Name senderName;
        Address addressSender;

        Name receiverName;
        Address addressReceiver;


    public:

        // Default constructor
        Package();

        // Main constructor
        Package(TypePackage type, float weight)
        {
            this->type = type;
            this->weight = weight;
        }

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
};