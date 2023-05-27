import re

def extract_function_declarations(c_code):
    function_declarations = []
    pattern = r'^\s*((?:\w+\s*\*?\s*)+\w+)\s*(\([^{}]*\))\s*{'
    matches = re.findall(pattern, c_code, re.MULTILINE)
    for match in matches:
        declaration = ' '.join(filter(None, match)).strip()+';'
        function_declarations.append(declaration)
    return function_declarations

# Read the C code from a file
with open("ui2c.c", "r") as file:
    c_code = file.read()

# Extract function declarations
declarations = extract_function_declarations(c_code)

# Print the function declarations
for declaration in declarations:
    print(declaration)
