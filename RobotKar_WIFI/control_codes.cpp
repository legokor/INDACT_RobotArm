/**
 *******************************************************************************
 * @file control_codes.cpp
 *
 * @date Jan 22, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief
 * This file implements the functions of "control_codes.h".
 *******************************************************************************
 */

#include "control_codes.h"

RequestType matchRequest(String req) {
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
    case 'z':
      switch (direction) {
        case 'm':
          return RequestType::Z_MINUS;
        case 'p':
          return RequestType::Z_PLUS;
        default:
          return RequestType::INVALID;
      }
    case 'f':
      switch (direction) {
        case 'm':
          return RequestType::PHI_MINUS;
        case 'p':
          return RequestType::PHI_PLUS;
        default:
          return RequestType::INVALID;
      }
    case 'r':
      switch (direction) {
        case 'm':
          return RequestType::R_MINUS;
        case 'p':
          return RequestType::R_PLUS;
        default:
          return RequestType::INVALID;
      }
    case 'h':
      if (direction != 'o') {
        return RequestType::INVALID;
      }
      return RequestType::HOME;
    default:
      return RequestType::INVALID;
  }
}

void requestToMessage(RequestType type, char *str) {
  switch (type) {
    case RequestType::Z_PLUS:
      strcpy_P(str, PSTR("zp\r\n"));
      break;
    case RequestType::Z_MINUS:
      strcpy_P(str, PSTR("zm\r\n"));
      break;
    case RequestType::PHI_PLUS:
      strcpy_P(str, PSTR("fp\r\n"));
      break;
    case RequestType::PHI_MINUS:
      strcpy_P(str, PSTR("fm\r\n"));
      break;
    case RequestType::R_PLUS:
      strcpy_P(str, PSTR("rp\r\n"));
      break;
    case RequestType::R_MINUS:
      strcpy_P(str, PSTR("rm\r\n"));
      break;
    case RequestType::HOME:
      strcpy_P(str, PSTR("ho\r\n"));
      break;
    default:
      strcpy_P(str, PSTR("00\r\n"));
  }
}
