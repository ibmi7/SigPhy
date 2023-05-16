
from serial import Serial
from serial.threaded import ReaderThread, Protocol, LineReader
#import pykst as kst
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
import math
import sys
import os
import binascii
from datetime import datetime
import time
import tkinter as tk


class SerialReaderProtocolLine(LineReader):
    tk_listener = None
    TERMINATOR = b'\n'

    def connection_made(self, transport):
        """Called when reader thread is started"""
        if self.tk_listener is None:
            raise Exception("tk_listener must be set before connecting to the socket!")
        super().connection_made(transport)
        print("Connected, ready to receive data...")

    def handle_line(self, line):
        """New line waiting to be processed"""
        # Execute our callback in tk
        print(line)
        self.tk_listener.after(0, self.tk_listener.on_data, line)

class MainFrame(tk.Frame):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.master.title("SigPhy")
        #self.master.geometry("800x600")
        self.pack()
        self.start_button = tk.Button(self, text="Start", command=self.start)
        self.start_button.pack()
        self.stop_button = tk.Button(self, text="Stop", command=self.stop)
        self.stop_button.pack()
        self.quit_button = tk.Button(self, text="Quit", command=self.quit)
        self.quit_button.pack()
        self.leds = ["led1","led2","led3","led4"]
        self.leds_value = [1000,1000,1000,1000]
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        # init parameters
        self.d = np.float32(4.32/1000)
        self.e_hb02_l1 = np.float32(464.5)
        self.e_hb02_l2 = np.float32(1159.3)
        self.e_hb02_l3 = np.float32(334.5)
        self.e_hb02_l4 = np.float32(1313.4)

        self.e_hb_l1 = np.float32(1295.6)
        self.e_hb_l2 = np.float32(785.9)
        self.e_hb_l3 = np.float32(3439.9)
        self.e_hb_l4 = np.float32(866.8)

        self.e_methb_l1 = np.float32(168.8)
        self.e_methb_l2 = np.float32(530.9)
        self.e_methb_l3 = np.float32(609.4)
        self.e_methb_l4 = np.float32(890.9)

        self.e_cco_l1 = np.float32(1800.5)
        self.e_cco_l2 = np.float32(2289.9)
        self.e_cco_l3 = np.float32(4399.8)
        self.e_cco_l4 = np.float32(1958.9)

        self.dpf_r_l1 = np.float32(6.21)
        self.dpf_r_l2 = np.float32(6.08)
        self.dpf_r_l3 = np.float32(5.00)
        self.dpf_r_l4 = np.float32(3.56)

        self.delta_a_l1 = np.float32(0.0)
        self.delta_a_l2 = np.float32(0.0)
        self.delta_a_l3 = np.float32(0.0)
        self.delta_a_l4 = np.float32(0.0)

        self.delta_hb02 = np.float32(0.0)
        self.delta_hb = np.float32(0.0)
        self.delta_methb = np.float32(0.0)
        self.delta_cco = np.float32(0.0)

        self.spo2_init = 96
        self.spo2 = 95
        self.delta_spo2 = 0.0
        self.matrice_data = np.array([[self.e_hb02_l1,self.e_hb_l1,self.e_methb_l1,self.e_cco_l1],[self.e_hb02_l2,self.e_hb_l2,self.e_methb_l2,self.e_cco_l2],[self.e_hb02_l3,self.e_hb_l3,self.e_methb_l3,self.e_cco_l3],[self.e_hb02_l4,self.e_hb_l4,self.e_methb_l4,self.e_cco_l4]],dtype=np.float32)
        self.mesure = np.array([[self.delta_a_l1 / self.dpf_r_l1],[self.delta_a_l2 / self.dpf_r_l2],[self.delta_a_l3 / self.dpf_r_l3],[self.delta_a_l4 / self.dpf_r_l4]],dtype=np.float32)
        self.resultat = np.array([[self.delta_hb02],[self.delta_hb],[self.delta_methb],[self.delta_cco]],dtype=np.float32)

    def start(self):
        self.fig, ax = plt.subplots(7, figsize=(20,20)) # créer une figure et un axe
        ecg = np.zeros(100)
        self.line1, = ax[0].plot(ecg, label = "ECG") # tracer une ligne aléatoire pour initialiser le graphique
        ax[0].set_ylim([0,3000]) # définir les limites de l'axe y
        self.line2, = ax[1].plot(np.random.randn(100),label = "Delta HbO2") # tracer une ligne aléatoire pour initialiser le graphique
        self.line3, = ax[2].plot(np.random.randn(100),label = "Delta Hb") # tracer une ligne aléatoire pour initialiser le graphique
        self.line4, = ax[3].plot(np.random.randn(100),label = "Delta MetHb") # tracer une ligne aléatoire pour initialiser le graphique
        self.line5, = ax[4].plot(np.random.randn(100),label = "Delta CCO") # tracer une ligne aléatoire pour initialiser le graphique
        self.line6, = ax[5].plot(np.random.randn(100),label = "SpO2") # tracer une ligne aléatoire pour initialiser le graphique
        self.line7, = ax[6].plot(np.random.randn(100),label = "Delta SpO2") # tracer une ligne aléatoire pour initialiser le graphique
        for ax in ax:
            ax.legend()
        #self.fig = Figure(figsize=(6,6))
        #a = self.fig.add_subplot(111) # créer une figure et un axe
        #self.line1, = a.plot(ecg, label = "ECG") # Returns a tuple of line objects, thus the comma
        #a.set_ylim([0,3000]) # définir les limites de l'axe y
        #b = self.fig.add_subplot(111)
        #self.line2 = b.bar(self.leds,self.leds_value, label = "LEDs", color = ["red","green","blue","yellow"])
        canva = FigureCanvasTkAgg(self.fig, master=self)
        canva.get_tk_widget().pack()
        canva.draw()
        # Set listener to our reader
        SerialReaderProtocolLine.tk_listener = self
        # Initiate serial port
        serial_port = Serial('COM9', 115200)
        # Initiate ReaderThread
        self.reader = ReaderThread(serial_port, SerialReaderProtocolLine)
        # Start reader
        self.reader.start()
        # Send start command
        self.reader.write("start".encode())


    def stop(self):
        print("stop")
        self.reader.write("stop".encode())
        self.reader.close() # fermer le port série
        self.destroy()
        self.__init__()




    def on_data(self, data):
        """Called when new data is received"""
        print(data)
        if "ecg" in data:
            data = data.replace("ecg","")
            a = int(data)
            self.line1.set_ydata(np.append(self.line1.get_ydata()[1:],a))
            print(a)
        elif "led1" in data:
            data = data.replace("led1","")
            a = int(data)
            #self.line2[0].set_height(a)
            self.delta_a_l1 = np.float32(-math.log10(a/self.leds_value[0]))
            print(a)
        elif "led2" in data:
            data = data.replace("led2","")
            a = int(data)
            #self.line2[1].set_height(a)
            self.delta_a_l2 = np.float32(-math.log10(a/self.leds_value[1]))
            print(a)
        elif "led3" in data:
            beg = time.time()
            data = data.replace("led3","")
            a = int(data)
            #self.line2[2].set_height(a)
            self.delta_a_l3 = np.float32(-math.log10(a/self.leds_value[2]))
            print(time.time() - beg)
        elif "led4" in data:
            beg = time.perf_counter()
            data = data.replace("led4","")
            a = int(data)
            #self.line2[3].set_height(a)
            self.delta_a_l4 = np.float32(-math.log10(a/self.leds_value[3]))
            self.mesure = np.array([[self.delta_a_l1 / self.dpf_r_l1],[self.delta_a_l2 / self.dpf_r_l2],[self.delta_a_l3 / self.dpf_r_l3],[self.delta_a_l4 / self.dpf_r_l4]],dtype=np.float32)
            self.resultat = (1/self.d) * np.dot(self.matrice_data,self.mesure)
            self.spo2 = 100 * (self.resultat[0,0] / (self.resultat[0,0] + self.resultat[1,0] + self.resultat[2,0]))
            self.delta_spo2 = self.spo2_init - self.spo2
            self.spo2_init = self.spo2
            self.line2.set_ydata(np.append(self.line2.get_ydata()[1:], self.resultat[0,0])) # ajouter la nouvelle valeur à la fin de la ligne
            self.line3.set_ydata(np.append(self.line3.get_ydata()[1:], self.resultat[1,0])) # ajouter la nouvelle valeur à la fin de la ligne
            self.line4.set_ydata(np.append(self.line4.get_ydata()[1:], self.resultat[2,0])) # ajouter la nouvelle valeur à la fin de la ligne
            self.line5.set_ydata(np.append(self.line5.get_ydata()[1:], self.resultat[3,0])) # ajouter la nouvelle valeur à la fin de la ligne
            self.line6.set_ydata(np.append(self.line6.get_ydata()[1:], self.spo2)) # ajouter la nouvelle valeur à la fin de la ligne
            self.line7.set_ydata(np.append(self.line6.get_ydata()[1:], self.delta_spo2)) # ajouter la nouvelle valeur à la fin de la ligne
            print(time.perf_counter() - beg)
        elif "stop" in data:
            self.quit()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def on_closing(self):
        self.reader.write("stop".encode())
        self.reader.close() # fermer le port série
        self.quit()


        

if __name__ == '__main__':
    app = tk.Tk()

    main_frame = MainFrame()
    # Set listener to our reader
    app.mainloop()


#start plotting function
""" def start_thread():
    global stop
    stop = False
    print("start")
    ser.write("start".encode())
    fig = Figure(figsize=(6,6))
    ecg = np.zeros(100)
    a = fig.add_subplot(111) # créer une figure et un axe
    line1, = a.plot(ecg, label = "ECG") # Returns a tuple of line objects, thus the comma
    a.set_ylim([0,3000]) # définir les limites de l'axe y
    canva = FigureCanvasTkAgg(fig, master=window)
    canva.get_tk_widget().pack()
    canva.draw()
    while True:
        ser2.flushInput()
        data = ser2.readline()
        if "ecg".encode() in data:
            data = data.replace("ecg".encode(),"".encode())
            a = int(data)
            line1.set_ydata(np.append(line1.get_ydata()[1:],a))
            fig.canvas.draw()
            fig.canvas.flush_events()
            print(a)
        elif "led1".encode() in data:
            data = data.replace("led1".encode(),"".encode())
            a = int(data)
            print(a)
        elif "led2".encode() in data:
            data = data.replace("led2".encode(),"".encode())
            a = int(data)
            print(a)
        elif "stop".encode() in data:
            break
        if stop:
            break

def stop_thread():
    print("stop")
    global stop
    stop = True
    ser.write("stop".encode())
    ser.close() # fermer le port série
 """