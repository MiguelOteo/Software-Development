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

class Package
{
    private:
        float weight;             ///< Weight of the package.
        Name senderName;          ///< Name of the sender.
        Address addressSender;    ///< Address of the sender.
        Name receiverName;        ///< Name of the receiver.
        Address addressReceiver;  ///< Address of the receiver.

    public:
        /**
         * @brief Main constructor for Package.
         * 
         * @param weight The weight of the package.
         */
        Package(float weight);

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
         * @brief Get the name of the sender.
         * 
         * @return The name of the sender.
         */
        Name getSenderName();

        /**
         * @brief Set the name of the sender.
         * 
         * @param senderName The new name of the sender.
         */
        void setSenderName(const Name &senderName);

        /**
         * @brief Get the address of the sender.
         * 
         * @return The address of the sender.
         */
        Address getAddressSender();

        /**
         * @brief Set the address of the sender.
         * 
         * @param addressSender The new address of the sender.
         */
        void setAddressSender(const Address &addressSender);

        /**
         * @brief Get the name of the receiver.
         * 
         * @return The name of the receiver.
         */
        Name getReceiverName();

        /**
         * @brief Set the name of the receiver.
         * 
         * @param receiverName The new name of the receiver.
         */
        void setReceiverName(const Name &receiverName);

        /**
         * @brief Get the address of the receiver.
         * 
         * @return The address of the receiver.
         */
        Address getAddressReceiver();

        /**
         * @brief Set the address of the receiver.
         * 
         * @param addressReceiver The new address of the receiver.
         */
        void setAddressReceiver(const Address &addressReceiver);

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
