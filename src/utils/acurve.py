# script generating A-weight curve coefficients based on formula from: https://en.wikipedia.org/wiki/A-weighting
# run: python a-curve-gen.py

import math

sampling_freq = 16000.0
coefficients_number = 512
step = sampling_freq / coefficients_number
results_array = []

print('// this file is generated by python script a-curve-gen.py')
print('// a-weight curve size ', coefficients_number,' coefficients with ', step, 'Hz resolution')
print('// start: ', step, ' end: ', step * coefficients_number, 'step: ', step)
print('')

for i in range(1, coefficients_number):
    # f = sampling_freq * i / coefficients_number
    f = step * i
    # coeff = (12194**2 * f**4) / ( (f**2 + 20.6**2) * math.sqrt( (f**2 + 107.7**2) * (f**2 + 737.9**2) ) * (f**2 + 12194**2) )
    coeff = (12200**2 * f**4) / ( (f**2 + 20.6**2) * (f**2 + 12200**2) * math.sqrt( (f**2 + 107.7**2) * (f**2 + 737.9**2) ) )
    results_array.append(coeff)

# convert array to string
results_string = ', '.join([str(elem) for elem in results_array])

print('float a_weighting_curve[PDM_BUFF_FFT_NUM_FREQS] = {')
print(results_string)
print('};')
