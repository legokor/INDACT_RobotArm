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

#include "Command.hpp"

namespace robot_arm_wifi
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
         * @param array_size Size of the command array. This is the number of commands in the array.
         */
        CommandManager(const std::vector<Command> &command_array)
            : commands(command_array)
        {
        }

        /**
         * @brief Check if the message contains a command and execute the command with the given
         *          data.
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
         * @brief Check if the message contains a command.
         *
         * @param message The message to be checked.
         * @param command The command that was found. (Output)
         * @param command_data The data for the execution of the command. (Output)
         * @return True if the message is a command, false otherwise.
         */
        bool MatchCommand(const std::string &message, Command &command, std::string &command_data) const
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

            for (auto command_local : this->commands)
            {
                if (command_local.Match(command_string))
                {
                    command = command_local;
                    command_data = data_string;
                    return true;
                }
            }

            return false;
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
        void AddCommand(const std::string &command_string, std::function<void(const std::string &)> handler)
        {
            this->AddCommand(Command(command_string, handler));
        }

        /**
         * @brief Add multiple commands to the manager.
         * 
         * @param commands The vector of commands to be added.
         */
        void AddCommands(const std::vector<Command> &commands)
        {
            for (auto command : commands)
            {
                this->AddCommand(command);
            }
        }

        /**
         * @brief Remove a command from the manager.
         *
         * @param command_string The string representation of the command.
         */
        void RemoveCommand(const std::string &command_string)
        {
            for (auto it = this->commands.begin(); it != this->commands.end(); ++it)
            {
                if (it->Match(command_string))
                {
                    this->commands.erase(it);
                    return;
                }
            }
        }

    private:
        std::vector<Command> commands;

    }; // class CommandManager

} // namespace robot_arm_wifi

#endif // COMMANDMANAGER_HPP_
