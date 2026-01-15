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

## Node

### Purpose

To get a config file, and to get from .ts to the compiled config, we need to execute some typescript code. This is where node comes in.

### Installation

```bash
# Download and install Homebrew
curl -o- https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh | bash

# Download and install Node.js:
brew install node@24

# Verify the Node.js version:
node -v # Should print "v24.12.0".

# Verify npm version:
npm -v # Should print "11.6.2".
```

OR GO TO
https://nodejs.org/en/download

Make sure to select the npm as "with" unless you know what you are doing

## Docker

### Purpose

Many of the scripts you will build for the backend REQUIRE Docker to be installed because thats how they compile into executable files

### Installation (mac)

https://docs.docker.com/desktop/setup/install/mac-install/

### Installation (windows)

https://docs.docker.com/desktop/setup/install/windows-install/

### Installation (linux)

https://docs.docker.com/desktop/setup/install/linux/

## Protobuf

### Purpose

Protobuf is like the middle sauce in between the different raspberry Pis. It is essentially a compiled language and also needs you to compile it in some scenarios.

### Installation

```bash
brew install protobuf
protoc --version # should be >=33.1
```

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

The main project is built with python so you will need the appropriate python stuffs

### Installation

```bash
brew install python3
python3 --version # Should print a version >= "3.12.6"
# Example output:
# Python 3.12.6
```

## Rust (OPTIONAL BUT _STRONGLY_ RECOMMENDED)

### Purpose

Rust is one of the bigger tools that you can use with this project for the backend. It's essentially a replacement c++.

### Installation

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup install beta
```

OR

https://rust-lang.org/tools/install/

## Thrift (Strongly Recommended)

### Purpose

Thrift is our language of config. I don't think you should need to compile it into anything with a dedicated command, but if you do, this is why you need this.

### Installation

```bash
brew install thrift
thrift --version # should be >=0.22.0
```

## Make Tools (Recommended)

### Purpose

Make Tools are a set of tools that are used to build the project. They are used to build the project into an executable file.

### Installation

```bash
brew install make
make --version
```

# Setting Up Workspace

## Initialize the submodules

```bash
git submodule update --init --recursive
```

## Install the dependencies

```bash
npm install
```

## Setup Python Virtual Environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## Install the dependencies

```bash
pip install -r requirements.txt
```

## Generate Files

```bash
make generate
npm run generate-thrift
```

## Build the project

```bash
./gradlew build
```

## Notes and Help:

- make sure that each of the following steps succeed without errors. If you get errors, work on each one accordingly before moving on to the next step.
