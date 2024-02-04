#!/bin/bash

# Check if the file path is provided as an argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <commands_file>"
    exit 1
fi

commands_file="$1"

# Check if the file exists
if [ ! -f "$commands_file" ]; then
    echo "Error: File '$commands_file' not found."
    exit 1
fi

# Read and execute commands from the file
while IFS= read -r command; do
    if [ -n "$command" ]; then
        echo "Executing: $command"
        eval "$command"
        status=$?

        if [ $status -ne 0 ]; then
            echo "Error: Command exited with status $status"
            exit 1
        fi
    fi
done < "$commands_file"

echo "All commands executed successfully."

