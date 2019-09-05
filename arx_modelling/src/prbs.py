#!/usr/bin/env python3

import matplotlib.pyplot as plt
import random

def get_rundom_number():
# Return the next random floating point number in the range [0.0, 1.0)
    return 1 if random.random() > 0.5 else -1

def prbs_sequence(N,band,range):

    # N = 10
    # band = 5
    sequence = []
    full = False
    step = 1
    previous_num = 1 #has to start with some value
    while (not full):
        current_num = get_rundom_number()
        # set the signal constant over 'band' interval
        if (current_num != previous_num):
            while(step <= band):
                if (N == 0):
                    full = True
                    step = band + 1 #to get out of the loop
                else:
                    sequence.append(current_num)
                    N -= 1
                    step += 1
            step = 1 #reset step
        else:
            if (N == 0):
                full = True
            else:
                sequence.append(current_num)
                N -= 1

        previous_num = current_num
    
    prbs_sequence = [i*range for i in sequence]
    return prbs_sequence



# test:
# sequence = prbs(25,2,1)

# print(sequence)

# plt.plot(sequence)
# plt.ylabel('prbs')
# plt.show()