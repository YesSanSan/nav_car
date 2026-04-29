# AGENTS.md

This repository is an STM32 H750 application project. AI coding agents must follow the project-local build workflow below.

## Build Rule

- Always build this project through the VS Code default build task.
- The default build task is defined in `.vscode/tasks.json` and is currently `H750: Build App`.
- Do not hand-write or directly run build commands such as `cmake`, `ninja`, `make`, or equivalent manual build invocations.
