/**
 * @file Package.cpp
 * @brief Implementation of the Package class, representing a parcel with sender and receiver information.
 * 
 * This file defines the main constructor and accessor/mutator functions for the Package class. 
 * The class encapsulates details about a package, including sender and receiver names, addresses, 
 * and the package weight.
 * 
 * @authors Miguel Oteo, Alvaro Redondo 
 */

#include <cmath>
#include <iostream>
#include "../../include/obj/Package.hpp"
#include "../../include/str/structs.hpp"

/**
 * @brief Main constructor for Package.
 * 
 * @param weight The weight of the package.
*/
Package::Package(float weight)
{
    this->weight = weight;
}

/*
* Getters and Setters
*/

/**
 * @brief Get the weight of the package.
 * 
 * @return The weight of the package.
 */
float Package::getWeight()
{
    return this->weight;
}

/**
 * @brief Set the weight of the package.
 * 
 * @param weight The new weight of the package.
 */
void Package::setWeight(float weight)
{
    this->weight = weight;
}

/**
 * @brief Get the name of the sender.
 * 
 * @return The name of the sender.
 */
Name Package::getSenderName()
{
    return this->senderName;
}

/**
 * @brief Set the name of the sender.
 * 
 * @param senderName The new name of the sender.
*/
void Package::setSenderName(const Name& senderName)
{
    this->senderName = senderName;
}

/**
 * @brief Get the address of the sender.
 * 
 * @return The address of the sender.
 */
Address Package::getAddressSender()
{
    return this->addressSender;
}

/**
 * @brief Set the address of the sender.
 * 
 * @param addressSender The new address of the sender.
 */
void Package::setAddressSender(const Address& addressSender)
{
    this->addressSender = addressSender;
}

/**
 * @brief Get the name of the receiver.
 * 
 * @return The name of the receiver.
 */
Name Package::getReceiverName()
{
    return this->receiverName;
}

/**
 * @brief Set the name of the receiver.
 * 
 * @param receiverName The new name of the receiver.
 */
void Package::setReceiverName(const Name& receiverName)
{
    this->receiverName = receiverName;
}

/**
 * @brief Get the address of the receiver.
 * 
 * @return The address of the receiver.
 */
Address Package::getAddressReceiver()
{
    return this->addressReceiver;
}

/**
 * @brief Set the address of the receiver.
 * 
 * @param addressReceiver The new address of the receiver.
 */
void Package::setAddressReceiver(const Address& addressReceiver)
{
    this->addressReceiver = addressReceiver;
}
