/**
 *******************************************************************************
 * @file control_codes.h
 *
 * @date Jan 22, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This file provides ways to process an incoming request from a client,
 *        and to communicate with the controller.
 *******************************************************************************
 */

#ifndef CONTROL_CODES_H_
#define CONTROL_CODES_H_

#include <stdint.h>
#include <WString.h>

/**
 * @brief Enum for the request types
 */
enum class RequestType {
  INVALID,
  INDEX,
  A_PLUS,
  A_MINUS,
  B_PLUS,
  B_MINUS,
  C_PLUS,
  C_MINUS,
  HOME,
  CHANGE_COORDINATES
};

/**
 * @brief Returns the request type of a given request string.
 * @param req A string containing the request
 * @return Type of the request, can be anything specified in the RequestType
 *        enum
 */
RequestType matchRequest(const String& req);

/**
 * @brief Prepares a message containing the type of a request.
 * @param type Type of the request
 * @param str Place of the new message
 */
void requestToMessage(RequestType type, char* str);

#endif /* CONTROL_CODES_H_ */