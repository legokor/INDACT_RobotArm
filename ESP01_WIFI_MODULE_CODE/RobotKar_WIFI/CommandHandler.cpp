/**
 ***************************************************************************************************
 * @file CommandHandler.cpp
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of CommandHandler.h.
 ***************************************************************************************************
 */

#include "CommandHandler.h"

using namespace std;
using namespace ESP8266_Controller;

CommandHandler::CommandHandler(const Command command_array[], size_t array_size)
    : commandArray(command_array), arraySize(array_size)
{
    // Compile the regex for matching the command and its data
    ex_command = "(\\w+)(?: (.*))?";
}

CommandHandler::~CommandHandler()
{
}

bool CommandHandler::matchCommand(const char *message)
{
    // Match the regex
    cmatch matches;
    if (!regex_match(message, matches, ex_command))
    {
        return false;
    }

    // Get the command string
    const char *command_string = matches[1].str().c_str();

    // Find the command
    for (size_t i = 0; i < arraySize; i++)
    {
        const Command *command = getCommandByIndex(i);
        if (strcmp(command_string, command->command_string) == 0)
        {
            lastMatchedCommand = command;
            lastMatchedCommandData = matches[2].str();
            return true;
        }
    }

    return false;
}

void CommandHandler::executeCommand(void)
{
    if (lastMatchedCommand != nullptr)
    {
        lastMatchedCommand->handler(lastMatchedCommandData.c_str());
    }
}

const Command *CommandHandler::getCommandByIndex(size_t index)
{
    // Check if the index is valid
    if (index >= arraySize)
    {
        // Return NULL if the index is invalid
        return NULL;
    }

    // Return the command
    return &(commandArray[index]);
}
