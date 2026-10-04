# Description

This will help you bootstrap this project into a new macbook laptop.

# Download package manager

Brew will be used for almost every since big installation from inside the terminal from now on

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

OR go to
https://brew.sh/

# Download Deps

## Java

### Purpose

The main project is built with java so you will need the appropriate java stuffs

### Installation

```bash
brew install java
java -version # Should print a version >= "17"
# Example output:
# java version "22.0.2" 2024-07-16
# Java(TM) SE Runtime Environment (build 22.0.2+9-70)
# Java HotSpot(TM) 64-Bit Server VM (build 22.0.2+9-70, mixed mode, sharing)
```

## Python3

### Purpose

The Gradle build runs a python script that clones and builds the dynamic vendor libraries

### Installation

```bash
brew install python3
python3 --version # Should print a version >= "3.12.6"
# Example output:
# Python 3.12.6
```

## Make Tools (Recommended)

### Purpose

Make Tools are a set of tools that are used to build the project. They are used to build the project into an executable file.

### Installation

```bash
brew install make
make --version
```

# Setting Up Workspace (inside the project directory)

1) Open up the terminal and make sure to go into the project directory. EG: `cd ~/Documents/robotics/2026-robot`. Using cd will allow you to go into the directory of the project. Then, in the same terminal, run the following commands.

## Build the project

This should build the project.

```bash
./gradlew build
```

## Notes and Help:

- make sure that each of the following steps succeed without errors. If you get errors, work on each one accordingly before moving on to the next step.
- Tell AI to look at this file and show it the error you are getting. Follow it's output and try to fix. 
