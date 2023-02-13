/**
 *******************************************************************************
 * @file control_codes.cpp
 *
 * @date Jan 22, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This file implements the functions of "control_codes.h".
 *******************************************************************************
 */

#include "control_codes.h"
#include "protocols.h"

RequestType matchRequest(const String& req) {
  if (req.indexOf("/ ") != -1) {
    return RequestType::INDEX;
  }

  int index = req.indexOf("mov=");
  if (index == -1) {
    return RequestType::INVALID;
  }

  char axis = req[index + 4];
  char direction = req[index + 5];
  switch (axis) {
    case 'a':
      switch (direction) {
        case 'm':
          return RequestType::A_MINUS;

        case 'p':
          return RequestType::A_PLUS;

        default:
          return RequestType::INVALID;
      }

    case 'b':
      switch (direction) {
        case 'm':
          return RequestType::B_MINUS;

        case 'p':
          return RequestType::B_PLUS;

        default:
          return RequestType::INVALID;
      }

    case 'c':
      switch (direction) {
        case 'm':
          return RequestType::C_MINUS;

        case 'p':
          return RequestType::C_PLUS;

        default:
          return RequestType::INVALID;
      }

    case 'h':
      if (direction != 'x') {
        return RequestType::INVALID;
      }

      return RequestType::HOME;

    case 'k':
      if (direction != 'x') {
        return RequestType::INVALID;
      }

      return RequestType::CHANGE_COORDINATES;

    default:
      return RequestType::INVALID;
  }
}

void requestToMessage(RequestType type, char* str) {
  switch (type) {
    case RequestType::A_PLUS:
      strcpy_P(str, PSTR(STR_AXIS_A_PLUS));
      break;

    case RequestType::A_MINUS:
      strcpy_P(str, PSTR(STR_AXIS_A_MINUS));
      break;

    case RequestType::B_PLUS:
      strcpy_P(str, PSTR(STR_AXIS_B_PLUS));
      break;

    case RequestType::B_MINUS:
      strcpy_P(str, PSTR(STR_AXIS_B_MINUS));
      break;

    case RequestType::C_PLUS:
      strcpy_P(str, PSTR(STR_AXIS_C_PLUS));
      break;

    case RequestType::C_MINUS:
      strcpy_P(str, PSTR(STR_AXIS_C_MINUS));
      break;

    case RequestType::HOME:
      strcpy_P(str, PSTR(STR_HOMING));
      break;

    case RequestType::CHANGE_COORDINATES:
      strcpy_P(str, PSTR(STR_CHANGE_COORDINATES));
      break;

    default:
      break;
  }
}
