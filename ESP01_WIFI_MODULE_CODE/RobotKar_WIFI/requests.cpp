/**
 ***************************************************************************************************
 * @file control_codes.cpp
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of requests.h.
 ***************************************************************************************************
 */

#include "requests.h"

#include <regex>

using namespace std;

/**
 * @brief Gets the request type from a given request.
 * @param request String vector containing the request parts
 * @return Request type
 */
RequestType getRequestType(const vector<string> &request);

/**
 * @brief Extracts the parts of a string using a given delimiter.
 * @param parts String vector to store the parts in
 * @param str String to extract the parts from
 * @param delimiter Character to use as delimiter
 */
void extractParts(vector<string> &parts, string str, char delimiter);

RequestType matchRequest(const string &request, string &parameters)
{
    // Patterns to use for the regular expression
    string pattern_resource = "((?:\\/[^\\/\\s]*)+)";
    string pattern_query_string = "\\??([^\\s]*)";
    string pattern_http_header = "^GET\\s+" + pattern_resource + pattern_query_string + "\\s+HTTP\\/\\d\\.\\d$";

    // Regular expression object
    regex ex_http_header(pattern_http_header);

    // Match results
    smatch match;
    if (!regex_search(request, match, ex_http_header))
    {
        // No match, return invalid
        return RequestType::INVALID;
    }

    // Extract captured parts
    string uri = match[1];
    string query = match[2];

    // Split URI into parts using the / character as delimiter
    vector<string> uri_parts;
    extractParts(uri_parts, uri, '/');

    // Get the request type
    RequestType type = getRequestType(uri_parts);

    parameters = "";

    // Get the parameters
    if (type == RequestType::ACTION)
    {
        vector<string> query_parts;
        // Split query string into parameters using the & character as delimiter
        extractParts(query_parts, query, '&');

        // Add the parameters to the parameters string
        for (int i = 1; i < uri_parts.size(); i++)
        {
            parameters += uri_parts[i];
            parameters += ' ';
        }
        for (int i = 0; i < query_parts.size(); i++)
        {
            parameters += query_parts[i];
            parameters += ' ';
        }
    }

    // Return the request type
    return type;
}

RequestType getRequestType(const vector<string> &request)
{
    // Check the first part of the request
    if ((request[0] == "") || (request[0] == "index"))
    {
        return RequestType::INDEX;
    }
    else if (request[0] == "data")
    {
        return RequestType::DATA;
    }
    else if (request[0] == "action")
    {
        // Check the second part of the request
        if (request.size() < 2)
        {
            // No second part, return invalid
            return RequestType::INVALID;
        }

        return RequestType::ACTION;
    }

    // No match, return invalid
    return RequestType::INVALID;
}

void extractParts(vector<string> &parts, string str, char delimiter)
{
    // Split string into parts using the delimiter
    size_t pos = 0;
    while ((pos = str.find(delimiter)) != string::npos)
    {
        // Add the part to the vector
        string part = str.substr(0, pos);
        parts.push_back(part);
        // Remove the part from the string
        str.erase(0, pos + 1);
    }
    parts.push_back(str); // add last part
}