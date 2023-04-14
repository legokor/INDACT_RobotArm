/**
 ***************************************************************************************************
 * @file requests.h
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file provides ways to process an incoming request from a client.
 ***************************************************************************************************
 */

#ifndef REQUESTS_H_
#define REQUESTS_H_

#include <string>

/**
 * @brief Enum for the request types
 */
enum class RequestType
{
    INVALID, /**< Invalid request */
    INDEX,   /**< Index request */
    DATA,    /**< Data request */
    ACTION   /**< Action request */
};

/**
 * @brief Extracts the request type and parameters from a given request.
 * @param request A string containing the first line of an HTTP request
 * @param parameters A string to store the parameters of the request in
 * @return Type of the request, can be anything specified in the RequestType enum
 */
RequestType matchRequest(const std::string &request, std::string &parameters);

#endif /* REQUESTS_H_ */