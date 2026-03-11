# AGENTS

## Verified workflows

- Workspace bootstrap: `git submodule update --init --recursive` then `npm install`
- Full robot build: `./gradlew build`
  - Also runs dynamic vendor dependency build (`scripts/clone_and_build_repos.py --config-file-path config.ini`) and protobuf generation.
- Java build wrapper: `make build`
- Java simulation: `./gradlew simulateJava`
- Robot deploy: `./gradlew deploy -PteamNumber=<TEAM_NUMBER>`
- Robot deploy wrapper: `make deploy` (uses `TEAM_NUMBER`, default `4765`)
- Combined deploy flow: `./gradlew deployAll`
  - Runs robot deploy plus backend deploy task.
- Gradle backend deploy task: `./gradlew deployBackend`
  - Invokes `make deploy-backend` and validates deployed Pi count against `EXPECTED_NUM_OF_PIS`.
- Python backend deploy directly: `make deploy-backend`
- Python test flow: `make initialize` (creates `.venv`, installs `requirements.txt`, then runs tests) and `make test`
- Config generation from TypeScript: `npm run config -- --dir src/config`
- Regenerate Thrift TS bindings: `npm run generate-thrift`
- Generate backend code artifacts: `make generate` (Python protobuf + Python thrift + Java proto task)
- Java protobuf generation via Gradle: `./gradlew generate` (depends on `generateProto`)

## Command notes

- `make build` and `make deploy` enforce Java 17 via `/usr/libexec/java_home -v 17`.
- `make deploy` defaults `TEAM_NUMBER=4765` unless overridden.
- `./gradlew deployBackend` expects deployment on `EXPECTED_NUM_OF_PIS` (currently `2`) and fails if mismatch.
- Dynamic vendor dependency builds are intentionally forced every Gradle compile/build cycle (`buildDynamicDeps.outputs.upToDateWhen { false }`).

## TODO

- Confirm whether automation should keep `TEAM_NUMBER=4765` and `EXPECTED_NUM_OF_PIS=2` as defaults or document per-robot override policy.
- README references `applyBackend` and `make prep-project`; verify whether docs should be updated to `deployBackend`/`make initialize`.

## Agent docs

- Lighting API and extension guide: `docs/LightingApiForAgents.md`
