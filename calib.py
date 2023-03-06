import numpy as np 
import pandas as pd 
import matplotlib.pyplot as plt 

calib = pd.read_csv('../calib.csv', header=None)

filename = input('Input filename: ')
pp = pd.read_csv(filename, header=None)

for i in range(len(pp)):
    for j in range(6):
        row = 0
        tmp = pp.iloc[i, j]
        if -90 <= tmp <= -80:
            low = -90
            high = -80
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
        
        row = row + 1
        if -80 <= tmp <= -70:
            low = -80
            high = -70
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if -70 <= tmp <= -60:
            low = -70
            high = -60
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
  
        row = row + 1
        if -60 <= tmp <= -50:
            low = -60
            high = -50
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
        
        row = row + 1
        if -50 <= tmp <= -40:
            low = -50
            high = -40
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
        row = row + 1

        if -40 <= tmp <= -30:
            low = -40
            high = -30
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
        row = row + 1

        if -30 <= tmp <= -20:
            low = -30
            high = -20
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if -20 <= tmp <= -10:
            low = -20
            high = -10
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]
 
        row = row + 1
        if -10 <= tmp <= 0:
            low = -10
            high = 0
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 0 <= tmp <= 10:
            low = 0
            high = 10
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 10 <= tmp <= 20:
            low = 10
            high = 20
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 20 <= tmp <= 30:
            low = 20
            high = 30
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 30 <= tmp <= 40:
            low = 30
            high = 40
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 40 <= tmp <= 50:
            low = 40
            high = 50
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 50 <= tmp <= 60:
            low = 50
            high = 60
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 60 <= tmp <= 70:
            low = 60
            high = 70
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 70 <= tmp <= 80:
            low = 70
            high = 80
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        row = row + 1
        if 80 <= tmp <= 90:
            low = 80
            high = 90
            anc_low = calib.iloc[row, j]
            anc_high = calib.iloc[row+1, j]

        pp.iloc[i, j] = anc_low + (anc_high - anc_low)*(tmp - low)/(high - low)

output = input('Input output file name: ')
pp.to_csv('../'+output, header=False, index=False)



