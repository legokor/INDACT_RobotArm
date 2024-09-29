# Project Guidelines

## Version Control

### Branching

- `main` is the main branch
- Use other branches for development, and merge them into the main when they are ready

### Committing

- On the main branch, always use meaningful commit messages
- On other branches, meaningful commit messages are only needed when working with others. Otherwise, use whatever you want, although it is still recommended to use meaningful commit messages.

### Pull Requests

- Pull requests are required for merging branches into the main branch
- Pull requests should be reviewed by at least one other person before merging

### Git Tools and IDE Integration

- Use whatever you want, but it is recommended to use a GUI tool for git, such as [VS Code Built-in Git](https://code.visualstudio.com/docs/sourcecontrol/overview "Using Git source control in VS Code") or the STM32CubeIDE Extension [EGit](https://eclipse.dev/egit/ "EGit - Git Integration for Eclipse").

## Project Structure

### File Structure

- The git repository contains the entire STM32CubeIDE project folder.
- Build files are ignored via `.gitignore`. Avoid including files that contain absolute paths, as this will cause issues when working with others. If you find that a file that should be ignored is not, please add it to the `.gitignore` file.
- Source files that are related to the same feature and can be well separated from others should be placed in the same folder, separated from other features. For example, all files related to the motor control should be placed in the `MotorControl` folder.
- Periphery initialization files should be generated as separate .c and .h file pairs, and placed in the `Core` folder.
- External libraries should be placed in the `Libraries` folder.

### Coding Conventions

- Use doxygen style comment blocks to document types, functions, and variables. Write the documentation only once.
- Use the FreeRTOS API for OS related functionality.
