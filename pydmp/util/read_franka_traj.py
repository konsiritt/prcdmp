
import csv
import numpy as np
import scipy.interpolate

def read_traj(path, labeled= False):

    with open(path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        dim = None
        odd = False
        traj = None
        traj = []
        for row in csv_reader:
            if line_count == 0 :
                odd = len(row)%2!=0
                dim = len(row)

            traj.append(row[:dim])
            line_count += 1
        #print(f'Processed {line_count} lines.')
        traj = np.transpose(traj)
        #print(traj)

        return traj
