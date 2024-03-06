/**
 * @file Package.hpp
 * @brief Header file for the Package class.
 * 
 * This file defines the Package class, which represents a package with information
 * such as weight, sender details, and receiver details. It also includes pure virtual
 * functions for calculating the cost, making it an abstract base class.
 * 
 * @authors Miguel Oteo, Alvaro Redondo
*/

#ifndef PACKAGE_HPP
#define PACKAGE_HPP

#include "../str/structs.hpp"
#include "Costumer.hpp"

class Package
{
    private:
        float weight;        ///< Weight of the package.
        Customer* sender;    ///< Sender customer of the package.
        Customer* receiver;  ///< Receiver customer of the package.

    public:
        /**
         * @brief Main constructor for Package.
         * 
         * @param weight The weight of the package.
         */
        Package(float weight, Customer* sender, Customer* receiver);

        /*
        * Getters and Setters
        */

        /**
         * @brief Get the weight of the package.
         * 
         * @return The weight of the package.
         */
        float getWeight();

        /**
         * @brief Set the weight of the package.
         * 
         * @param weight The new weight of the package.
         */
        void setWeight(float weight);

        /**
         * @brief Get the sender.
         * 
         * @return The customer who sends the package.
         */
        Customer* getSender();

        /**
         * @brief Set the sender.
         * 
         * @param sender The new customer who sends the package.
         */
        void setSender(Customer* sender);

        /**
         * @brief Get the reciever.
         * 
         * @return The customer who recieves the package.
         */
        Customer* getReceiver();

        /**
         * @brief Set the receiver.
         * 
         * @param receiver The new customer who Receives the package.
         */
        void setReceiver(Customer* receiver);

        /**
         *  Methods
         */

        /**
         * @brief Virtual function to calculate the cost of a package.
         * 
         * This function should be overridden by derived classes.
         * 
         * @return The total cost of the package.
         */
        virtual float calculateCost() = 0;
};

#endif // PACKAGE_HPP
