## PWRUP Command Robot Base

### What this repo is

This is a command-based FRC robot starter that glues together:

- **WPILib 2026 + GradleRIO** for the main Java robot code
- **AdvantageKit** for logging, replay, and sim workflows
- **Dynamically built vendor libraries** pulled from source at build time

The intent is simple: clone this once per robot, drop in your subsystems/commands, and ship.

---

## Why you might use this

- **You want one place to start every new robot.** Command-based scaffold, logging, and vendor deps are already wired.
- **You tweak vendor code a lot.** Libraries like `PWRUPCore` and `SwerveDrive` can be built straight from their Git repos on every build.
- **You care about logs.** AdvantageKit is already integrated for REAL/SIM/REPLAY modes.

---

## Prerequisites

- **Java 17** (required by WPILib 2026)
- **Python 3.9+** (runs the dynamic vendor build script)

---

## Quick start

1. **Clone the base**

```bash
git clone <this-repo> my-robot
cd my-robot
```

2. **Choose which vendor libraries build from source**

Edit `config.ini` to match the libraries and branches you want:

```ini
[PWRUPCore]
build_dynamically = true
github = https://github.com/PinewoodRobotics/PWRUPCore.git
branch = main
force_clone = false
```

On `./gradlew build`, any section with `build_dynamically = true` is:

- cloned into `lib/vendor/`
- built with that repo’s own Gradle build
- copied into `lib/build/` and wired into the Java classpath

3. **Build, simulate, and deploy**

```bash
# Full Java build + dynamic vendor builds
./gradlew build

# WPILib simulator GUI
./gradlew simulateJava

# Deploy to your RoboRIO
./gradlew deploy -PteamNumber=<TEAM_NUMBER>
```

---

## Project layout (high level)

- `src/main/java/frc/robot` – main robot code (`Robot`, `RobotContainer`, constants, subsystems, commands)
- `lib/vendor` – source checkouts of dynamically built vendor libraries
- `lib/build` – JARs produced from those vendors and added to the Java classpath

If you want more detail on the dynamic source-building system, see `docs/SourceBuildingPlugin.md`.

---

## Logging modes

`BotConstants` controls how AdvantageKit runs:

- **REAL** – logs written on the roboRIO
- **SIM** – NT4 publisher plus GUI
- **REPLAY** – reads a `.wplog` file and writes a new `_sim` log

Switch between REAL and SIM in `BotConstants` when running off-roboRIO.

---

## Common commands

```bash
# Full build (Java + dynamic vendors)
./gradlew build

# Clean out dynamically built vendor outputs
rm -rf lib/vendor lib/build
```

---

## Using this as your base

The expectation is that you **clone or fork this once per robot**, then:

- keep the Gradle pieces mostly as-is
- add robot-specific subsystems, commands, and constants

If you find improvements to the base itself, open a pull request with a short justification and keep the pieces modular and easy to reason about.
