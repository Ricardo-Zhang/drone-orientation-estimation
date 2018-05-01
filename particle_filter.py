import numpy as np
import matplotlib.pyplot as plt


def load_data():
    path = 'data/'
    gps_loc_file = 'gps_Locations.npy'
    gps_loc_noisy_file = 'Observations.npy'
    drone_loc_file = 'Drone_Locations.npy'
    drone_orie_file = 'Drone_Orientations.npy'
    gps_location = np.load(path+gps_loc_file)
    measurements = np.load(path+gps_loc_noisy_file)
    drone_location = np.load(path+drone_loc_file)
    ground_truth = np.load(path+drone_orie_file)
    return gps_location, measurements, drone_location, ground_truth


def main():
    _, measurements, _, ground_truth = load_data()
    baseline_matrix = np.zeros((3, 3))
    for i in range(3):
        baseline_matrix[i] = measurements[0][-1] - measurements[0][i]
    print(baseline_matrix)


if __name__ == "__main__":
    main()