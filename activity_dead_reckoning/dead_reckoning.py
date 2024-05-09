import numpy as np
import pprint
from dead_reckoning_class import DeadReckoning

def format_array(arr):
    formatted_string = np.array2string(arr, precision=4, floatmode='fixed', separator=', ', suppress_small=True)
    formatted_string = formatted_string.replace('.0000,', ',')  # Remove decimal point if unnecessary
    return formatted_string

robot = DeadReckoning()  # Initial position (all zeros)
num_steps = 2  # Number of simulation steps

# Initial conditions
u0 = robot.u0
E0 = robot.E0

print(f"Initial conditions:")
print(f"μ{0} =\n {u0}")
print(f"Σ{0} =\n {E0}")
print("==========================")

for i in range(num_steps):
    # Calculate the following variables
    hk = robot.linearisation()
    uk = robot.estimated_position()
    Ek = robot.propagation_uncertainity(hk)

    uk_formatted = format_array(uk)
    hk_formatted = format_array(hk)
    Ek_formatted = format_array(Ek)

    print(f"Step {i+1}:")
    print(f"Estimated position\n μ{i+1} =\n {uk_formatted}")
    print(f"Linearised model\n H{i+1} =\n {hk_formatted}")
    print(f"Propagation of the uncertainity \n Σ{i+1} =\n {Ek_formatted}")
    print("==========================")
