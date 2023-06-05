"""
UART Service
-------------

An example showing how to write a simple program using the Nordic Semiconductor
(nRF) UART service.

"""

import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

# plot stuff
import matplotlib.pyplot as plt
import time
import numpy as np
import numpy.fft as fft
import drawnow

from scipy.signal import butter, lfilter
from scipy.signal import savgol_filter

import math
import csv

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))

# init parameters
d = np.float32(4.32/1000)
e_hb02_l1 = np.float32(464.5)
e_hb02_l2 = np.float32(982.6)
e_hb02_l3 = np.float32(334.5)
e_hb02_l4 = np.float32(1313.4)

e_hb_l1 = np.float32(1295.6)
e_hb_l2 = np.float32(950.9)
e_hb_l3 = np.float32(3439.9)
e_hb_l4 = np.float32(866.8)

e_methb_l1 = np.float32(168.8)
e_methb_l2 = np.float32(795.1)
e_methb_l3 = np.float32(609.4)
e_methb_l4 = np.float32(890.9)

e_cco_l1 = np.float32(1800.5)
e_cco_l2 = np.float32(1176.1)
e_cco_l3 = np.float32(4399.8)
e_cco_l4 = np.float32(1958.9)

dpf_t_l1 = np.float32(3.49)
dpf_t_l2 = np.float32(3.25)
dpf_t_l3 = np.float32(2.80)
dpf_t_l4 = np.float32(2.00)

matrice_data = np.array([[e_hb02_l1,e_hb_l1,e_methb_l1,e_cco_l1],[e_hb02_l2,e_hb_l2,e_methb_l2,e_cco_l2],[e_hb02_l3,e_hb_l3,e_methb_l3,e_cco_l3],[e_hb02_l4,e_hb_l4,e_methb_l4,e_cco_l4]],dtype=np.float32)

def calc_spo2(l1, l2, l3, l4):
    mesure = np.array([[l1 / dpf_t_l1],[l2 / dpf_t_l2],[l3 / dpf_t_l3],[l4 / dpf_t_l4]],dtype=np.float32)
    resultat = (1/d) * (matrice_data @ mesure)
    spo2 = 100 * (resultat[0,0] / (resultat[0,0] + resultat[1,0] + resultat[2,0]))
    return spo2, resultat
    

cnt = 0 # Compteur du nombre de points
beg = 0 # Début du chronomètre
spo2 = 95
async def uart_terminal():
    """This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    temps = [0] # Liste des temps
    LED1 = [1.5]
    LED2 = [1.5]
    LED3 = [1.5]
    LED4 = [1.5]
    with open("data.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["TEMPS","LED1","LED2","LED3","LED4", "Delta HbO2", "Delta Hb", "Delta MetHb", "Delta CCO", "SPO2", "Delta SPO2"])
        #writer.writerow(["TEMPS", "ECG"])
        file.flush()
        
    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        global beg
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            beg = time.time() # Début du chronomètre
            return True

        return False

    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()
        file.close()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        global cnt
        global beg
        global spo2
        #print("received:", data)
        # delete bytearray from data
        #s = data.replace('bytearray(b'.encode(),''.encode())
        #print(data[0:3])
        #print(data[3:6])
        a = int((data[0:3].hex()),16)
        b = int((data[3:6].hex()),16)
        c = int((data[6:9].hex()),16)
        d = int((data[9:12].hex()),16)
        #print(a)
        #print(b)
        data1 = int(a) / 8388608.0 * 4.0
        data2 = int(b) / 8388608.0 * 4.0
        data3 = int(c) / 8388608.0 * 4.0
        data4 = int(d) / 8388608.0 * 4.0
        LED1.append(data1)
        LED2.append(data2)
        LED3.append(data3)
        LED4.append(data4)
        
        l1 = np.float32(-math.log10(data1/LED1[-2]))
        l2 = np.float32(-math.log10(data2/LED2[-2]))
        l3 = np.float32(-math.log10(data3/LED3[-2]))
        l4 = np.float32(-math.log10(data4/LED3[-2]))
        prev_spo2 = spo2
        spo2, resultat = calc_spo2(l1, l2, l3, l4)
        delta_spo2 = spo2 - prev_spo2
        temps.append(time.time()-beg)
        cnt += 1
        
        print("LED1 : " + str(data1))
        print("LED2 : " + str(data2))
        print("LED3 : " + str(data3))
        print("LED4 : " + str(data4))
        ''' print("received:", data)
        print(data[0:7])
        a = data[0:8].decode()
        a = ''.join(filter(str.isdigit, a))
        a = int(a)
        if a & (1 << 23):
            a = a | ~((1 << 24) - 1)
            
        ecg = int(a) / 8388608.0 * 4.0
        print("ECG : " + str(ecg))'''
        # write to file
        #file.write(str(time.time()-beg) + ";" + str(LED1) + ";" + str(LED2) + "\n")
        
        # find treshold (90 % of max value)
        # get sampling frequency (1 / mean of time between two points)
            # apply low pass filter to data
            # Filter requirements.
        if cnt > 100:
            order = 6
            #fs = 1 / np.mean(np.diff(temps[-100:]))
            fs = 100
            cutoff = 10  # desired cutoff frequency of the filter, Hz
            print(fs)
            # Get the filter coefficients so we can check its frequency response.
            b, a = butter(order, cutoff/(0.5*fs), btype= 'low', analog=False)
            y1 = lfilter(b, a, LED1[-100:])
            y2 = lfilter(b, a, LED2[-100:])
            y3 = lfilter(b, a, LED3[-100:])
            y4 = lfilter(b, a, LED4[-100:])
            yhat1 = savgol_filter(y1, 51, 3) # window size 51, polynomial order 3
            yhat2 = savgol_filter(y2, 51, 3) # window size 51, polynomial order 3
            yhat3 = savgol_filter(y3, 51, 3) # window size 51, polynomial order 3
            yhat4 = savgol_filter(y4, 51, 3) # window size 51, polynomial order 3
            
            """maxi1 = max(yhat1)
            mini1 = min(yhat1)
            tres1 = mini1 + 0.9 * (maxi1 - mini1)
            tres_min1 = mini1 + 0.1 * (maxi1 - mini1)
            maxi2 = max(yhat2)
            mini2 = min(yhat2)
            tres2 = mini2 + 0.9 * (maxi2 - mini2)
            tres_min2 = mini2 + 0.1 * (maxi2 - mini2)
            maxi3 = max(yhat3)
            mini3 = min(yhat3)
            tres3 = mini3 + 0.9 * (maxi3 - mini3)
            tres_min3 = mini3 + 0.1 * (maxi3 - mini3)
            maxi4 = max(yhat4)
            mini4 = min(yhat4)
            tres4 = mini4 + 0.9 * (maxi4 - mini4)
            tres_min4 = mini4 + 0.1 * (maxi4 - mini4)
            print("Treshold : " + str(tres1))
            #create array with only values above treshold along with their time stamp
            # find index of values above treshold
            ind1 = np.array([])
            ind2 = np.array([])
            ind3 = np.array([])
            ind4 = np.array([])
            flag = 1
            for i in range(len(yhat1)):
                if y1[-500:][i] > tres1 and flag == 1:
                    ind1 = np.append(ind1, i)
                    flag = 0
                elif y1[-500:][i] < tres_min1 and flag == 0:
                    flag = 1
            
            flag = 1
            for i in range(len(yhat2)):
                if y2[-500:][i] > tres2 and flag == 1:
                    ind2 = np.append(ind2, i)
                    flag = 0
                elif y2[-500:][i] < tres_min2 and flag == 0:
                    flag = 1

            flag = 1
            for i in range(len(yhat3)):
                if y3[-500:][i] > tres3 and flag == 1:
                    ind3 = np.append(ind3, i)
                    flag = 0
                elif y3[-500:][i] < tres_min3 and flag == 0:
                    flag = 1

            flag = 1
            for i in range(len(yhat4)):
                if y4[-500:][i] > tres4 and flag == 1:
                    ind4 = np.append(ind4, i)
                    flag = 0
                elif y4[-500:][i] < tres_min4 and flag == 0:
                    flag = 1

            # find time stamp of values above treshold from temps array
            temp1 = np.array([])
            for i in ind1:
                temp1 = np.append(temp1, temps[-500:][int(i)])

            temp2 = np.array([])
            for i in ind2:
                temp2 = np.append(temp2, temps[-500:][int(i)])

            temp3 = np.array([])
            for i in ind3:
                temp3 = np.append(temp3, temps[-500:][int(i)])

            temp4 = np.array([])
            for i in ind4:
                temp4 = np.append(temp4, temps[-500:][int(i)])"""

            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(yhat1[-1]), str(yhat2[-1]), str(yhat3[-1]), str(yhat4[-1]), str(float(resultat[0,0])), str(float(resultat[1,0])), str(float(resultat[2,0])), str(float(resultat[3,0])), str(spo2), str(delta_spo2)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()
        else:
            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(data1), str(data2), str(data3), str(data4), str(float(resultat[0,0])), str(float(resultat[1,0])), str(float(resultat[2,0])), str(float(resultat[3,0])), str(spo2), str(delta_spo2)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()
        """if cnt > 500:
            # get sampling frequency (1 / mean of time between two points)
            # apply low pass filter to data
            # Filter requirements.
            order = 6
            fs = 1 / np.mean(np.diff(temps[-500:]))
            cutoff = 10  # desired cutoff frequency of the filter, Hz
            
            # Get the filter coefficients so we can check its frequency response.
            b, a = butter(order, cutoff/(0.5*fs), btype= 'low', analog=False)
            y = lfilter(b, a, LED1[-500:])
            yhat = savgol_filter(y, 51, 3) # window size 51, polynomial order 3

            
            maxi = max(yhat)
            mini = min(yhat)
            tres = mini + 0.9 * (maxi - mini)
            tres_min = mini + 0.1 * (maxi - mini)
            print("Treshold : " + str(tres))
            #create array with only values above treshold along with their time stamp
            # find index of values above treshold
            ind = np.array([])
            flag = 1
            for i in range(len(yhat)):
                if y[-500:][i] > tres and flag == 1:
                    ind = np.append(ind, i)
                    flag = 0
                elif y[-500:][i] < tres_min and flag == 0:
                    flag = 1
            # find time stamp of values above treshold from temps array
            temp = np.array([])
            for i in ind:
                temp = np.append(temp, temps[-500:][int(i)])
            
            # find average time between two peaks to find frequency
            freq = 1 / np.mean(np.diff(temp))
            print(temp)
            print("Frequency : " + str(freq))

            # write frequency to file
            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(yhat[-1]), str([LED2[-1]])])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()
            
            '''             maxi = max(LED1[-500:])
            mini = min(LED1[-500:])
            tres = mini + 0.9 * (maxi - mini)
            tres_min = mini + 0.1 * (maxi - mini)
            print("Treshold : " + str(tres))
            #create array with only values above treshold along with their time stamp
            # find index of values above treshold
            ind = np.array([])
            flag = 1
            for i in range(len(LED1[-500:])):
                if LED1[-500:][i] > tres and flag == 1:
                    ind = np.append(ind, i)
                    flag = 0
                elif LED1[-500:][i] < tres_min and flag == 0:
                    flag = 1
            # find time stamp of values above treshold from temps array
            temp = np.array([])
            for i in ind:
                temp = np.append(temp, temps[-500:][int(i)])
            
            # find average time between two peaks to find frequency
            freq = 1 / np.mean(np.diff(temp))
            print(temp)
            print("Frequency : " + str(freq))

            # write frequency to file
            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(data1), str(data2), str(freq)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()
            '''                
        else:
            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(data1), str(data2), str(0)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()"""
                
        


    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, start typing and press ENTER...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not data:
                break

            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            for s in sliced(data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s)

            print("sent:", data)


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass