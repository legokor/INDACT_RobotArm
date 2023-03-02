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

#include <regex>
#include <WString.h>

using namespace std;

RequestType matchRequest(const char *request)
{
    const regex ex_index("GET / .*");
    const regex ex_data("GET /data/? .*");
    const regex ex_action("GET /action/(\\w{2})/? .*");

    if (regex_match(request, ex_index))
    {
        return RequestType::INDEX;
    }

    if (regex_match(request, ex_data))
    {
        return RequestType::DATA;
    }

    cmatch match;
    if (regex_match(request, match, ex_action))
    {
        char axis = match[1].str()[0];
        char direction = match[1].str()[1];
        switch (axis)
        {
        case 'a':
            switch (direction)
            {
            case 'm':
                return RequestType::A_MINUS;

            case 'p':
                return RequestType::A_PLUS;

            default:
                return RequestType::INVALID;
            }

        case 'b':
            switch (direction)
            {
            case 'm':
                return RequestType::B_MINUS;

            case 'p':
                return RequestType::B_PLUS;

            default:
                return RequestType::INVALID;
            }

        case 'c':
            switch (direction)
            {
            case 'm':
                return RequestType::C_MINUS;

            case 'p':
                return RequestType::C_PLUS;

            default:
                return RequestType::INVALID;
            }

        case 'h':
            if (direction != 'x')
            {
                return RequestType::INVALID;
            }

            return RequestType::HOME;

        case 'k':
            if (direction != 'x')
            {
                return RequestType::INVALID;
            }

            return RequestType::CHANGE_COORDINATES;

        default:
            return RequestType::INVALID;
        }
    }

    return RequestType::INVALID;
}

void requestToMessage(RequestType type, char *str)
{
    switch (type)
    {
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
