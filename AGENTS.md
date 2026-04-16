# AGENTS

## Verified workflows

- Workspace bootstrap: `git submodule update --init --recursive` then `npm install`
- Full robot build: `./gradlew build`
  - Also runs dynamic vendor dependency build (`scripts/clone_and_build_repos.py --config-file-path config.ini`) and protobuf generation.
- Java build wrapper: `make build`
- Java test flow: `./gradlew test`
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
- Python protobuf generation only: `make generate-proto-python`
- Python thrift generation only: `make thrift-to-py`
- Java protobuf generation via Makefile: `make proto-to-java` (runs `./gradlew generateProto`)
- Java protobuf generation via Gradle: `./gradlew generate` (depends on `generateProto`)

## Command notes

- `make build` and `make deploy` enforce Java 17 via `/usr/libexec/java_home -v 17`.
- `make deploy` defaults `TEAM_NUMBER=4765` unless overridden.
- `./gradlew deployBackend` expects deployment on `EXPECTED_NUM_OF_PIS` (currently `3`) and fails if mismatch.
- `make generate` assumes `.venv` already exists because `generate-proto-python` runs `.venv/bin/fix-protobuf-imports`; use `make initialize` first on a fresh clone.
- `make test` also assumes `.venv` already exists because it executes `.venv/bin/python`; run `make initialize` first on a fresh clone.
- `npm run config` can omit `--dir` and auto-detect `config/` or `src/config/`; `json`, `json-binary`, and `file` switches are supported.
- Dynamic vendor dependency builds are intentionally forced every Gradle compile/build cycle (`buildDynamicDeps.outputs.upToDateWhen { false }`).

## TODO

- Confirm whether automation should keep `TEAM_NUMBER=4765` and `EXPECTED_NUM_OF_PIS=3` as defaults or document per-robot override policy.
- `README.md` and `docs/HowToInitialize.md` still reference older/manual backend setup (`applyBackend`, `make prep-project`, raw `pip install -r requirements.txt`); update docs to `deployBackend`/`make initialize` and mention `make generate`'s `.venv` dependency.
- Recent `src/backend/python/pos_extrapolator/__tests__` growth has no dedicated top-level target yet; decide whether to add/document a focused pytest workflow (currently routed through `make test`).

## Agent docs

- Lighting API and extension guide: `docs/LightingApiForAgents.md`
