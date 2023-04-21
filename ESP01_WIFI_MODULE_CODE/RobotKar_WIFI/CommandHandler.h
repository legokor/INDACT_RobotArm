/**
 ***************************************************************************************************
 * @file CommandHandler.h
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file contains the declaration of the CommandHandler class and related data types.
 ***************************************************************************************************
 */

#ifndef COMMANDHANDLER_H_
#define COMMANDHANDLER_H_

#include <regex>

namespace ESP8266_Controller
{

    /**
     * @brief Handler function of a command.
     */
    typedef void (*CommandHandlerFunction)(const char *);

    /**
     * @brief Structure of a command.
     */
    typedef struct
    {
        const char *command_string;     /**< String of the command */
        CommandHandlerFunction handler; /**< Handler function of the command */
    } Command;

    /**
     * @brief Class for handling commands.
     */
    class CommandHandler
    {
    private:
        /** @brief Array of commands. */
        const Command *commandArray;
        /** @brief Size of the command array. */
        const size_t arraySize;
        /** @brief Regular expression of the command. */
        std::regex ex_command;
        /** @brief Regular expression of the command data. */
        const Command *lastMatchedCommand = nullptr;
        /** @brief Last matched command data. */
        std::string lastMatchedCommandData;

    public:
        /**
         * @brief Constructor.
         * @param command_array Array of commands
         * @param array_size Size of the command array
         */
        CommandHandler(const Command command_array[], size_t array_size);

        /**
         * @brief Destructor.
         */
        ~CommandHandler();

        /**
         * @brief Check if the message is a command.
         * @param message The message to be checked in string format
         * @return True if the message is a command, false otherwise
         */
        bool matchCommand(const char *message);

        /**
         * @brief Execute the last matched command.
         */
        void executeCommand(void);

    private:
        /**
         * @brief Get the command by index.
         * @param index Index of the command
         * @return Pointer to the command
         */
        const Command *getCommandByIndex(size_t index);

    }; /* class CommandHandler */

} /* namespace ESP8266_Controller */

#endif /* COMMANDHANDLER_H_ */