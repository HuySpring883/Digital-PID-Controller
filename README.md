# Digital-PID-Controller

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction
TODO

## Notes
- controlling DC moter
- process of sampling; converting continuous-time signal into a sequence of numbers.
- process of data reconstruction; converting a sequence of numbers into a continuous-time signal.
- micro-controller board is used to perform processes; D/A converters, optical encoder
- optical encoder used to handle digital outputs of a integer counter; produces angular position of motor's load shaft

## DLaB Library Functions
- library of C functions used to simplify simulation and implementation of digital controlers
- functions had 2 modes 1. Simulation 2. Hardware
- Software to hardware testing requires no code modification
- 
## Features
List the main features of your project. This could include:
- Digital PID control design using Ultimate Sensitivity Method
- Simulation functions used for testing
- Concurrent programming with Pthreads
- Real-time control of a physical DC motor hardware

## Installation
Provide step-by-step instructions on how to set up and install your project. Include any prerequisites and dependencies. For example:

```bash
# Clone the repository
git clone https://github.com/HuySpring883/Digital-PID-Controller.git

# Navigate to the project directory
cd Digital-PID-Controller

# Install dependencies
# (List any specific commands or packages needed)
```

## Usage
Explain how to use your project. Include examples and command-line instructions. For example:

```bash
# Compile the code
make

# Run the PID controller
./pid_controller
```

## Project Structure
Describe the structure of your project. For example:

```
.
├── src
│   ├── main.c
│   ├── pid.c
│   └── pid.h
├── tests
│   ├── test_pid.c
│   └── test_motor.c
├── docs
│   └── manual.md
├── Makefile
└── README.md
```

## Examples
Provide some example scenarios or use cases. Include code snippets and explanations.

## Contributing
Explain how others can contribute to your project. Include guidelines for submitting issues and pull requests.

## License
Specify the license under which your project is distributed. For example:

```
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
```

## Acknowledgements
Give credit to any individuals, libraries, or resources that helped you with your project.

```
Special thanks to [Name] for their assistance in developing this project.
```

---
