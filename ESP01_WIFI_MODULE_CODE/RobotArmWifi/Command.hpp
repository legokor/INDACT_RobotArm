#ifndef COMMAND_HPP_
#define COMMAND_HPP_

/**
 * @file Command.hpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the implementation of the Command class.
 * 
 */

#include <string>
#include <functional>

namespace robot_arm_wifi
{

    /**
     * @brief Class for handling commands.
     */
    class Command
    {
    public:
        /**
         * @brief Default constructor.
         */
        Command() = default;

        /**
         * @brief Constructor with command string and handler function initialization.
         *
         * @param command_string String of the command.
         * @param handler Handler function of the command (callback).
         */
        Command(const std::string &command_string, std::function<void(const std::string &)> handler)
            : commandString(command_string), commandHandler(handler)
        {
        }

        /**
         * @brief Copy constructor.
         *
         * @param other The other command object.
         */
        Command(const Command &other)
            : commandString(other.commandString), commandHandler(other.commandHandler)
        {
        }

        /**
         * @brief Get the command string.
         *
         * @return The command string.
         */
        const std::string &GetCommandString(void) const
        {
            return this->commandString;
        }

        /**
         * @brief Set the command string.
         *
         * @param command_string The command string.
         */
        void SetCommandString(const std::string &command_string)
        {
            this->commandString = command_string;
        }

        /**
         * @brief Get the command handler function.
         *
         * @return The command handler function.
         */
        const std::function<void(const std::string &)> &GetCommandHandler(void) const
        {
            return this->commandHandler;
        }

        /**
         * @brief Set the command handler function.
         *
         * @param command_handler The command handler function.
         */
        void SetCommandHandler(const std::function<void(const std::string &)> &command_handler)
        {
            this->commandHandler = command_handler;
        }

        /**
         * @brief Match the command string with the given string.
         *
         * @param command_string The string to be matched.
         * @return True if the command string matches the given string, false otherwise.
         */
        bool Match(const std::string &command_string) const
        {
            return (command_string == this->commandString);
        }

        /**
         * @brief Execute the command handler function.
         *
         * @param command_data The command data to be passed to the command handler function.
         */
        void Execute(const std::string &command_data) const
        {
            this->commandHandler(command_data);
        }

        /**
         * @brief Assignment operator.
         *
         * @param other The other command object.
         * @return This command object.
         */
        Command &operator=(const Command &other)
        {
            this->commandString = other.commandString;
            this->commandHandler = other.commandHandler;

            return *this;
        }

    private:
        std::string commandString;
        std::function<void(const std::string &)> commandHandler;

    }; // class Command

} // namespace robot_arm_wifi

#endif // COMMAND_HPP_
