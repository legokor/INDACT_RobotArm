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

      return RequestType::COORD_CHANGE;

    default:
      return RequestType::INVALID;
  }
}

void requestToMessage(RequestType type, char* str) {
  switch (type) {
    case RequestType::A_PLUS:
      strcpy_P(str, PSTR("ap\r\n"));
      break;

    case RequestType::A_MINUS:
      strcpy_P(str, PSTR("am\r\n"));
      break;

    case RequestType::B_PLUS:
      strcpy_P(str, PSTR("bp\r\n"));
      break;

    case RequestType::B_MINUS:
      strcpy_P(str, PSTR("bm\r\n"));
      break;

    case RequestType::C_PLUS:
      strcpy_P(str, PSTR("cp\r\n"));
      break;

    case RequestType::C_MINUS:
      strcpy_P(str, PSTR("cm\r\n"));
      break;

    case RequestType::HOME:
      strcpy_P(str, PSTR("hx\r\n"));
      break;

    case RequestType::COORD_CHANGE:
      strcpy_P(str, PSTR("kx\r\n"));
      break;

    default:
      strcpy_P(str, PSTR("00\r\n"));
  }
}
