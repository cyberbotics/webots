# PROTO formatter

The present script formats the indentations of the given PROTO file.

## Requirements

This script requires Python 3 to be installed and accessible from your terminal.

## Usage

```shell
python3 proto_formatter.py <path/to/proto/file.proto>
```

## Auto-format with VS-Code

It is possible to configure Visual Studio Code to run the script from the IDE directly.
To do so, follow these instuctions:

- In the editor, press `Ctrl+Maj+P`.
- Type "Tasks: Configure Default Build Task".
- Click on "Create tasks.json file from template" and then "Others".
- Replace the content by the following and save the file:

  ```json
  {
    "version": "2.0.0",
    "tasks": [
      {
        "label": "Format PROTO file",
        "type": "shell",
        "command": "python",
        "args": [
          "$(WEBOTS_HOME)/scripts/proto_formatter/proto_formatter.py",
          "${file}"
        ],
        "group": {
          "kind": "build",
          "isDefault": true
        },
        "problemMatcher": {
          "owner": "python",
          "fileLocation": ["relative", "${file}"],
          "pattern": {
            "regexp": ".*"
          }
        },
        "options": {
          "cwd": "${fileDirname}",
          "file": "${file}"
        },
        "presentation": {
          "reveal": "always",
          "panel": "shared"
        },
        "isBackground": true,
        "runOptions": {
          "runOn": "folderOpen"
        }
      }
    ]
  }
  ```

- Replace `$(WEBOTS_HOME)` by the path to your Webots installation folder.
- Press `Ctrl+Maj+B` to trigger the formatting in a .proto file.
- It is possible to automatically format .proto files when saving them with the extension: [Trigger Task on Save](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.triggertaskonsave).
- Once installed, you can add the following entry in the `settings.json` file (`Ctrl+,`)

  ```json
  "triggerTaskOnSave.tasks": {
    "Format PROTO file": [
      "*.proto",
    ],
  }
  ```
