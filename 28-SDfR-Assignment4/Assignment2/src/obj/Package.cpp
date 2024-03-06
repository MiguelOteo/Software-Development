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

#include "../../include/obj/Package.hpp"
#include "../../include/str/structs.hpp"

/**
 * @brief Main constructor for Package.
 * 
 * @param weight The weight of the package.
 */
Package::Package(float weight, Customer* sender, Customer* receiver) : 
    weight(weight), sender(sender), receiver(receiver){};

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
 * @brief Get the sender.
 * 
 * @return The customer who sends the package.
 */
Customer* Package::getSender()
{
    return this->sender;
}

/**
 * @brief Set the sender.
 * 
 * @param sender The new customer who sends the package.
 */
void Package::setSender(Customer* sender)
{
    this->sender = sender;
}

/**
 * @brief Get the reciever.
 * 
 * @return The customer who recieves the package.
 */
Customer* Package::getReceiver()
{
    return this->receiver;
}

/**
 * @brief Set the receiver.
 * 
 * @param receiver The new customer who Receives the package.
 */
void Package::setReceiver(Customer* receiver)
{
    this->receiver = receiver;
}
