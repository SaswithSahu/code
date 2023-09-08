# my_rover_code_wrapper.pyx

# Import the Python module directly
import my_rover_code

def call_generate_rover_code(float u1, float u2):
    # Call the Python function using the imported module
    result_tuple = my_rover_code.generate_rover_code(u1, u2)
    result1, result2 = result_tuple
    return result1, result2

