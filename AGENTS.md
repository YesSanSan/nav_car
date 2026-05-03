## Build Rule

- Build this project using the VS Code default build task, or by directly running the exact command defined by that task.
- The default build task is defined in `.vscode/tasks.json` and is currently `H750: Build App`.
- Direct command execution is allowed only when it is functionally identical to the VS Code task: same command, arguments, working directory, and required environment.
- Do not invent, simplify, or substitute build commands such as manual `cmake`, `ninja`, `make`, or equivalent commands unless they are exactly what the default task runs.
- If `.vscode/tasks.json` changes, the task definition is authoritative.
