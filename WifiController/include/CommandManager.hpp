#ifndef COMMANDMANAGER_HPP_
#define COMMANDMANAGER_HPP_

/**
 * @file CommandManager.hpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 *
 * @brief This file contains the implementation of the CommandManager class.
 *
 */

#include <string>
#include <vector>
#include <algorithm>

#include "Command.hpp"

namespace wifi_controller
{

    /**
     * @brief Class for managing commands.
     */
    class CommandManager
    {
    public:
        /**
         * @brief Default constructor.
         */
        CommandManager() = default;

        /**
         * @brief Constructor with command array initialization.
         *
         * @param command_array Pointer to the array of commands.
         */
        explicit CommandManager(const std::vector<Command> &command_array)
            : commands(command_array)
        {
        }

        /**
         * @brief Check if the message contains a command and execute the command with the given data.
         *
         * @param message The message to be checked.
         * @return True if the message is a command, false otherwise.
         */
        bool MatchAndExecuteCommand(const std::string &message) const
        {
            Command command;
            std::string command_data;

            if (this->MatchCommand(message, command, command_data))
            {
                command.Execute(command_data);
                return true;
            }

            return false;
        }

        /**
         * @brief Check if the message contains a command and return the matching command and the command data.
         *
         * @param message The message to be checked.
         * @param command (Output) The command that has a command string that matches the one found in the message.
         * @param command_data (Output) The data that is required for the execution of the command.
         * @return True if the message is a command, false otherwise.
         */
        [[nodiscard]] bool MatchCommand(const std::string &message, Command &command, std::string &command_data) const
        {
            std::string command_string, data_string;
            std::size_t command_end = message.find_first_of(' ');
            if (command_end == std::string::npos)
            {
                command_string = message;
                data_string = "";
            }
            else
            {
                command_string = message.substr(0, command_end);
                data_string = message.substr(command_end + 1);
            }

            auto it = std::find_if(
                this->commands.begin(), this->commands.end(),
                [&command_string](const Command &command)
                {
                    return command.Match(command_string);
                });

            if (it == this->commands.end())
            {
                return false;
            }

            command = *it;
            command_data = data_string;
            return true;
        }

        /**
         * @brief Add a command to the manager.
         *
         * @param command The command to be added.
         */
        void AddCommand(const Command &command)
        {
            this->commands.push_back(command);
        }

        /**
         * @brief Add a command to the manager.
         *
         * @param command_string The string representation of the command.
         * @param handler The handler function (callback) for the command.
         */
        void AddCommand(const std::string &command_string, const std::function<void(const std::string &)> &handler)
        {
            this->commands.push_back(Command(command_string, handler));
        }

        /**
         * @brief Add multiple commands to the manager.
         *
         * @param commands The vector of commands to be added.
         */
        void AddCommands(const std::vector<Command> &commands)
        {
            this->commands.insert(this->commands.end(), commands.begin(), commands.end());
        }

        /**
         * @brief Remove a command from the manager.
         *
         * @param command_string The string representation of the command.
         */
        void RemoveCommand(const std::string &command_string)
        {
            // This code is based on the erase–remove idiom.
            // More info: https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
            this->commands.erase(
                std::remove_if(
                    this->commands.begin(), this->commands.end(),
                    [&command_string](const Command &command)
                    {
                        return command.Match(command_string);
                    }),
                this->commands.end());
        }

    private:
        std::vector<Command> commands;

    }; // class CommandManager

} // namespace wifi_controller

#endif // COMMANDMANAGER_HPP_
