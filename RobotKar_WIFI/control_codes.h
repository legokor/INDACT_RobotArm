/**
 *******************************************************************************
 * @file control_codes.h
 *
 * @date Jan 22, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief
 * This file provides ways to process an incoming request from a client, and to
 * communicate with the controller.
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
  Z_PLUS,
  Z_MINUS,
  PHI_PLUS,
  PHI_MINUS,
  R_PLUS,
  R_MINUS,
  HOME
};

/**
 * @brief Returns the request type of a given request string.
 * @param req A string containing the request
 * @return Type of the request, can be anything specified in the RequestType
 *        enum
 */
RequestType matchRequest(String req);

/**
 * @brief Prepares a message containing the type of a request.
 * @param type Type of the request
 * @param str Place of the new message (the size has to be at least 5 char)
 */
void requestToMessage(RequestType type, char *str);

#endif /* CONTROL_CODES_H_ */