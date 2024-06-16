import numpy as np
from scipy import linalg
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import threading

class SensorCalibrator:
    def __init__(self):
        self.mag_b = np.zeros([3, 1])
        self.mag_A_1 = np.eye(3)
        self.accel_b = np.zeros([3, 1])
        self.accel_A_1 = np.eye(3)
        self.data = []
        self.stop_event = threading.Event()

    def run(self, mag_filename, accel_filename):
        mag_data = self.load_data(mag_filename)
        accel_data = self.load_data(accel_filename)

        if mag_data is None or accel_data is None:
            return

        print(f"Shape of magnetometer data:", mag_data.shape)
        print(f"First 5 rows raw magnetometer data:\n", mag_data[:5])

        print(f"Shape of accelerometer data:", accel_data.shape)
        print(f"First 5 rows raw accelerometer data:\n", accel_data[:5])
        
        self.mag_b, self.mag_A_1 = self.calibrate_sensor(mag_data, 46.1766)
        self.accel_b, self.accel_A_1 = self.calibrate_sensor(accel_data, 46.1766)
        
        self.plot_data(mag_data, "Raw Magnetometer Data")
        self.plot_data(accel_data, "Raw Accelerometer Data")
        
        calibrated_mag_data = self.calibrate_data(mag_data, self.mag_b, self.mag_A_1)
        calibrated_accel_data = self.calibrate_data(accel_data, self.accel_b, self.accel_A_1)
        
        self.plot_data(calibrated_mag_data, "Calibrated Magnetometer Data")
        self.plot_data(calibrated_accel_data, "Calibrated Accelerometer Data")
        
        print(f"First 5 rows calibrated magnetometer data:\n", calibrated_mag_data[:5])
        print(f"First 5 rows calibrated accelerometer data:\n", calibrated_accel_data[:5])
        
        np.savetxt(f'calibrated_{mag_filename}', calibrated_mag_data, fmt='%f', delimiter=',')
        np.savetxt(f'calibrated_{accel_filename}', calibrated_accel_data, fmt='%f', delimiter=',')
        
        self.print_calibration_parameters()

    def load_data(self, filename):
        try:
            data = np.loadtxt(filename, delimiter=',')
        except Exception as e:
            print(f"Error loading data from {filename}: {e}")
            return None

        if data.size == 0:
            print(f"No data found in {filename}")
            return None

        return data[:, :3]  # Extract only the relevant 3 columns

    def calibrate_sensor(self, data, F):
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        M_1 = linalg.inv(M)
        A_1 = np.real(F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        b = -np.dot(M_1, n)

        return b, A_1
    
    def calibrate_data(self, data, b, A_1):
        result = []
        for row in data:
            xm_off, ym_off, zm_off = row - b.flatten()
            xm_cal = xm_off * A_1[0,0] + ym_off * A_1[0,1] + zm_off * A_1[0,2]
            ym_cal = xm_off * A_1[1,0] + ym_off * A_1[1,1] + zm_off * A_1[1,2]
            zm_cal = xm_off * A_1[2,0] + ym_off * A_1[2,1] + zm_off * A_1[2,2]
            result.append([xm_cal, ym_cal, zm_cal])
        return np.array(result)
    
    def plot_data(self, data, title):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(data[:,0], data[:,1], data[:,2], marker='o')
        ax.set_title(title)
        plt.show()
    
    def print_calibration_parameters(self):
        print("Magnetometer Calibration Parameters:")
        print("const float magSoftIronMatrix[3][3] = {")
        print(f"    {{{self.mag_A_1[0,0]}, {self.mag_A_1[0,1]}, {self.mag_A_1[0,2]}}},")
        print(f"    {{{self.mag_A_1[1,0]}, {self.mag_A_1[1,1]}, {self.mag_A_1[1,2]}}},")
        print(f"    {{{self.mag_A_1[2,0]}, {self.mag_A_1[2,1]}, {self.mag_A_1[2,2]}}}")
        print("};")
        print("\nconst float magHardIronBias[3] = {")
        print(f"    {self.mag_b[0][0]}, {self.mag_b[1][0]}, {self.mag_b[2][0]}")
        print("};")

        print("\nAccelerometer Calibration Parameters:")
        print("const float accelSoftIronMatrix[3][3] = {")
        print(f"    {{{self.accel_A_1[0,0]}, {self.accel_A_1[0,1]}, {self.accel_A_1[0,2]}}},")
        print(f"    {{{self.accel_A_1[1,0]}, {self.accel_A_1[1,1]}, {self.accel_A_1[1,2]}}},")
        print(f"    {{{self.accel_A_1[2,0]}, {self.accel_A_1[2,1]}, {self.accel_A_1[2,2]}}}")
        print("};")
        print("\nconst float accelHardIronBias[3] = {")
        print(f"    {self.accel_b[0][0]}, {self.accel_b[1][0]}, {self.accel_b[2][0]}")
        print("};")

    def read_serial_data(self, _len=10000):
        self.ser = serial.Serial('COM4', 115200)
        self.ser.flushInput()
        progress = 0

        def stop_data_collection():
            input("Press Enter to stop data collection...\n")
            self.stop_event.set()

        threading.Thread(target=stop_data_collection).start()

        while not self.stop_event.is_set() and progress < _len:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if ',' in line:
                    try:
                        values = list(map(float, line.split(',')))
                        if len(values) == 6:
                            self.data.append(values)
                            progress += 1
                            print(f"Progress: {progress}/{_len}, {values}", end='\r')
                    except ValueError:
                        continue

        self.data = np.array(self.data)
        print("\nData collection complete.")
        self.ser.close()

        #First 3 columns are magnetometer data, next 3 columns are accelerometer data
        mag_data = self.data[:, :3]
        accel_data = self.data[:, 3:]
        np.savetxt("magnetometer_data.txt", mag_data, fmt='%f', delimiter=',')
        np.savetxt("accelerometer_data.txt", accel_data, fmt='%f', delimiter=',')
        return self.data

    def __ellipsoid_fit(self, s):
        D = np.array([s[0]**2., s[1]**2., s[2]**2., 2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1], 2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]
        C = np.array([[-1,  1,  1,  0,  0,  0], [ 1, -1,  1,  0,  0,  0], [ 1,  1, -1,  0,  0,  0], [ 0,  0,  0, -4,  0,  0], [ 0,  0,  0,  0, -4,  0], [ 0,  0,   0,  0,  0, -4]])
        E = np.dot(linalg.inv(C), S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))
        E_w, E_v = np.linalg.eig(E)
        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)
        M = np.array([[v_1[0], v_1[5], v_1[4]], [v_1[5], v_1[1], v_1[3]], [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]], [v_2[1]], [v_2[2]]])
        d = v_2[3]
        return M, n, d

if __name__ == '__main__':
    calibrator = SensorCalibrator()
    combined_data = calibrator.read_serial_data()
    calibrator.run("magnetometer_data.txt", "accelerometer_data.txt")
