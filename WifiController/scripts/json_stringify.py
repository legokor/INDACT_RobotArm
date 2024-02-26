import json
import os
import sys


def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <text_file>")
        sys.exit(1)

    file_name = sys.argv[1]

    try:
        # Read the text file.
        with open(file_name, "r") as file:
            # Convert the text file to a JSON string with special characters escaped.
            result = json.dumps(file.read())

    except FileNotFoundError:
        print(f"File not found: {file_name}")
        sys.exit(1)

    # Create the output file with "_string.txt" appended to the input file name.
    output_file_name = os.path.splitext(file_name)[0] + "_string.txt"

    # Write the result to the output file.
    with open(output_file_name, "w") as file:
        file.write(result)

    print(f"Output saved to: {output_file_name}")


if __name__ == "__main__":
    main()
