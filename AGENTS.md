# AGENTS

## Verified workflows

- Full robot build: `./gradlew build`
  - Also runs dynamic vendor dependency build (`scripts/clone_and_build_repos.py --config-file-path config.ini`).
- Java build wrapper: `make build`
- Java test flow: `./gradlew test`
- Java simulation: `./gradlew simulateJava`
- Robot deploy: `./gradlew deploy -PteamNumber=<TEAM_NUMBER>`
- Robot deploy wrapper: `make deploy` (uses `TEAM_NUMBER`, default `4765`)

## Command notes

- `make build` and `make deploy` enforce Java 17 via `/usr/libexec/java_home -v 17`.
- `make deploy` defaults `TEAM_NUMBER=4765` unless overridden.
- Dynamic vendor dependency builds are intentionally forced every Gradle compile/build cycle (`buildDynamicDeps.outputs.upToDateWhen { false }`).

## TODO

- Confirm whether automation should keep `TEAM_NUMBER=4765` as the default or document per-robot override policy.

## Agent docs

- Lighting API and extension guide: `docs/LightingApiForAgents.md`
