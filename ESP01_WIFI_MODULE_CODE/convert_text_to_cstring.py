import sys
import os


def text_to_cstring(input_file, output_file=None):
    # Read the text file
    with open(input_file, "r") as f:
        text = f.read()

    # Escape special characters
    text = text.replace('\\', '\\\\')
    text = text.replace('"', '\\"')
    text = text.replace('\n', '\\n\"\n\"')

    # Write the text to a file in C string format
    if output_file is None:
        output_file = os.path.splitext(input_file) + '_cstring.txt'
    with open(output_file, "w") as f:
        # f.write(f'static const char {os.path.splitext(os.path.basename(output_file))[0]}[] PROGMEM = "{text}";')
        f.write(f'"{text}"')


if __name__ == "__main__":
    # Check if the input file name was provided
    if len(sys.argv) < 2:
        print("Usage: python convert_text_to_cstring.py input_file.txt [output_file.txt]")
        sys.exit()

    # Convert text to C string
    input_file = sys.argv[1]
    output_file = None
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    text_to_cstring(input_file, output_file)
