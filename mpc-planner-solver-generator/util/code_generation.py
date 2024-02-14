
def tabs(tab_level):
    result = ""
    for i in range(tab_level):
        result += "\t"

    return result


def open_function(h_file, cpp_file, function_header, optional_header_with_defaults=None, class_name="Solver", has_type=True):

    if optional_header_with_defaults is None:
        optional_header_with_defaults = function_header

    # In the cpp file write the function declaration with function opener
    if has_type:
        split_header = function_header.split()
        idx = (len(split_header) > 1)
        split_header[idx] = class_name + "::" + split_header[idx]
        function_header = ' '.join(split_header)
    else:
        function_header = class_name + "::" + function_header # For the constructor

    cpp_file.write(tabs(1) + function_header)
    cpp_file.write("\n" + tabs(1) + "{\n")

    # In the h file, add the declaration only
    h_file.write(tabs(1) + optional_header_with_defaults + ";\n")


def close_function(file):
    file.write(tabs(1) + "}\n\n")

def add_zero_below_10(val, N) -> str:
    if N < 9:
        return str(val)
    
    if val < 10:
        return "0" + str(val)
    else:
        return str(val)