
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Thread
import numpy as np
from multilat_wpower import *
import serial
import serial.tools
import serial.tools.list_ports
from tracking import Kalman
from serial import Serial
from tkinter import Frame, Tk, Image, Button, Text, Canvas, PhotoImage, Label, Entry, Toplevel
from tkterminal import Terminal
from paho_mqtt_sub import *
from paho_mqtt_pub import *
from PIL import Image, ImageTk
from json import JSONDecodeError


BAUDRATE = 115200
DEVICE = '/dev/ttyACM0'
SQUARE_OFFSET = 10
SCREEN_REFRESH = 500 # Refresh every 500 ms

HOST = "csse4011-iot.zones.eait.uq.edu.au"
TOPIC = "un44333289"

GRAPH_OFFSET = 50

GRAPH_SIZE = 700 # 700x700 (Roughly)

MAX_LEN_ARENA = 4

OPTIONS = ["Name...", 
           "Addr...", 
           "Major...", 
           "Minor...", 
           "X...", 
           "Y...", 
           "Left Neighbour...", 
           "Right Neighbour..."]

class Redirect():
    
    def __init__(self, widget):
        self.widget = widget

    def write(self, text):
        self.widget.insert('end', text)
        self.widget.see('end') # autoscroll

    def flush(self):
        pass



@dataclass
class Beacon:

    name: str
    addr: str
    major: int
    minor: int
    x: float
    y: float
    left_neighbour: str
    right_neighbour: str

    def __repr__(self) -> str:
        return f"Name: {self.name}, Addr: {self.addr}, Major: {self.major}, Minor: {self.minor}, Pos: [{self.x}, {self.y}], Left Neighbour: {self.left_neighbour}, Right Neighbour: {self.right_neighbour}"
    


class DataFusionApp(Tk):

    def __init__(self):

        super().__init__()

        # Main window settings
        self.title("Data Fusion Visualiser")
        self.minsize(1200, 1000)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        # Window is split into the graphing and text box for debugging/displaying info.
        self.frame_display = Frame(master=self, width=1500, height=1000, background='black')
        self.frame_text = Frame(master=self, width=500, height=300, background='white')
        self.frame_text.grid(row=1, column=0)
        self.frame_display.grid(row=0, column=0, sticky='nesw')
        self.frame_display.rowconfigure(0, weight=1)
        self.frame_display.columnconfigure(0, weight=1)
        self.frame_display.rowconfigure(1, weight=1)
        self.frame_text.rowconfigure(0, weight=1)

        # Frame display is split into the graph and buttons
        self.display_graph = Frame(master=self.frame_display, width=800, height=800, bg='black')
        self.display_btns = Frame(master=self.frame_display, width=1000, height=50, background='black')
        self.display_graph.grid(row=0, column=0)
        self.display_btns.grid(row=1, column=0, sticky='ews', pady=5)
        self.display_graph.rowconfigure(0, weight=1)
        self.display_graph.columnconfigure(0, weight=1)
        self.display_btns.rowconfigure(0, weight=1)
        self.display_btns.columnconfigure(0, weight=1)
        self.display_btns.columnconfigure(1, weight=1)
        self.display_btns.columnconfigure(2, weight=1)
        
        # Buttons for the app
        self.add_button = Button(master=self.display_btns, background='orange', text='Add iBeacon', command=self._add_button_event)
        self.remove_button = Button(master=self.display_btns, background='orange', text='Remove iBeacon', command=self._remove_button_event)
        self.display_button = Button(master=self.display_btns, background='orange', text='Display iBeacon', command=self._display_beacon_info_event)
        self.add_button.grid(row=0, column=0, padx=100)
        self.remove_button.grid(row=0, column=1, padx=50)
        self.display_button.grid(row=0, column=2, padx=100)
      
        # Text box to display information to the user
        self.text_box = Text(master=self.frame_text, height=15, width=100)
        self.text_box.grid(sticky='ew')
        self.pos_label = Label(master=self.frame_display, 
                               background='black', 
                               height=2, 
                               width=20, 
                               text="Pos: 0.0 m, 0.0 m", 
                               fg="orange",)
        self.pos_label.place(x=20, y=600)
        self.motion_label = Label(master=self.frame_display, 
                                  background='black', 
                                  height=2, 
                                  width=20, 
                                  text="Vx: 0.0 Vy: 0.0", 
                                  fg="orange")
        self.motion_label.place(x=20, y=650)
        self.rssi_label = Label(master=self.frame_display, 
                                background='black', 
                                height=1, 
                                width=20, 
                                text="RSSI Ranging", 
                                fg="orange")
        self.rssi_label.place(x=20, y=700)
        self.rssi_label_vals = Label(master=self.frame_display, 
                                     background='black', 
                                     height=1, 
                                     width=30, 
                                     text="0 0 0 0 0 0 0 0", 
                                     fg="orange",
                                     font=('Helvetica bold', 7))
        self.rssi_label_vals.place(x=10, y=720)
        self.time_stamp = Label(master=self.frame_display, 
                                background='black', 
                                height=1, 
                                width=20, 
                                text="Timestamp", 
                                fg="orange",)
        self.time_stamp.place(x=20, y=750)
        self.time_stamp_val = Label(master=self.frame_display, 
                                background='black', 
                                height=1, 
                                width=20, 
                                text="0.0", 
                                fg="orange")
        self.time_stamp_val.place(x=20, y=770)
        

        # MQTT Connection
        # self.mqttc = self._connect_mqtt(HOST, TOPIC)

        # Setup serial connection
        self.serial = self._connect_device(DEVICE)

        # Generate grid image on the canvas
        self._canvas_setup()

        # Default list of beacons
        self.beacons: dict[Beacon] = self._generate_beacons()

        self.valid_beacons = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']

        # Kalman filter for RSSI
        self.ndim = 4
        self.ndim_obs = 2
        self.xcoord = 2.0
        self.ycoord = 2.0
        self.vx = 0.0 #m.s
        self.vy = 0.0 #m/s
        self.dt = 1.0 #sec
        self.meas_error_rssi = 0.5 #m
        self.meas_error_ultra = 0.06 #m

        #init filter
        self.proc_error = 0.1
        self.init_error = 10.0
        self.x_init = np.array( [self.xcoord+self.init_error, self.ycoord+self.init_error, self.vx, self.vy] ) #introduced initial xcoord error of 10m 
        cov_init=self.init_error*np.eye(self.ndim)
        
        self.k = Kalman(self.x_init, cov_init, self.meas_error_rssi, self.proc_error) 

        # Redirect stdout to the GUI textbox
        sys.stdout = Redirect(self.text_box)

        self.latest_serial_json: json = 0
        
        self.rssi_average = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.mqtt_on = 0

        self.cur_pos = (0.0, 0.0)

        self.current_points = []



    """ Internal function to generate a default beacon list, neighbours of the beacons
        are assumed to be looking towards the centre of the arena. """
    def _generate_beacons(self) -> dict[Beacon]:
        
        beacons = {}
        beacons["A"] = Beacon("4011-A", "F5:75:FE:85:34:67", 2753, 32998, 0, 0, "4011-B", "4011-H")
        beacons["B"] = Beacon("4011-B", "E5:73:87:06:1E:86", 32975, 20959, 2, 0, "4011-C", "4011-A")
        beacons["C"] = Beacon("4011-C", "CA:99:9E:FD:98:B1", 26679, 40363, 4, 0, "4011-D", "4011-B")
        beacons["D"] = Beacon("4011-D", "CB:1B:89:82:FF:FE", 41747, 38800, 4, 2, "4011-E", "4011-C")
        beacons["E"] = Beacon("4011-E", "D4:D2:A0:A4:5C:AC", 30679, 51963, 4, 4, "4011-F", "4011-D")
        beacons["F"] = Beacon("4011-F", "C1:13:27:E9:B7:7C", 6195, 18394, 2, 4, "4011-G", "4011-E")
        beacons["G"] = Beacon("4011-G", "F1:04:48:06:39:A0", 30525, 30544, 0, 4, "4011-H", "4011-F")
        beacons["H"] = Beacon("4011-H", "CA:0C:E0:DB:CE:60", 57395, 28931, 0, 2, '4011-A', "4011-G")
        beacons["I"] = Beacon("4011-I", "D4:7F:D4:7C:20:13", 60345, 49995, -1, -1, '', '')
        beacons["J"] = Beacon("4011-J", "F7:0B:21:F1:C8:E1", 12249, 30916, -1, -1, '', '')
        beacons["K"] = Beacon("4011-K", "FD:E0:8D:FA:3E:4A", 36748, 11457, -1, -1, '', '')
        beacons["L"] = Beacon("4011-L", "EE:32:F7:28:FA:AC", 27564, 27589, -1, -1, '', '')

        return beacons
    
    def _draw_point(self, x, y, colour):
    
        point = self.graph_canvas.create_rectangle((x-SQUARE_OFFSET, 
                                            y-SQUARE_OFFSET, 
                                            x+SQUARE_OFFSET, 
                                            y+SQUARE_OFFSET), 
                                            fill=colour)

        self.current_points.append(point)

    """ Reads from serial a serialized message (json), then decodes it
        and returns a json object. """
    def _serial_read(self) -> json:

        json_str = ''
        while True:

            try:
                char = self.serial.read(1).decode()
            except UnicodeDecodeError:
                print("Decoding Error occurred, resetting")
                char = ''

            if char == '{':
                # Beginning of our JSON string
                json_str = ''
            
            json_str += char
            if char == '}' and json_str[0] == '{':
                # end of the JSON string
                try:
                    json_format = json.loads(json_str)
                except JSONDecodeError:
                    print("JSON Error occurred")
                    json_str = ''
                    continue
                # print("RSSI:", json_format)
                # print("Timestamp:", time.time())
                return json_format
                

    def _handle_serial(self):

        # Continuously read the serial
        # and grab RSSI values to calculate position
        index = 0
        rssi_average_calc = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        while True:
            
            self.latest_serial_json = self._serial_read()

            #Timestamp
            a, b, c, d, e, f, g, h = self._json_to_list(self.latest_serial_json)
            self.rssi_label_vals.config(text="%d, %d, %d, %d, %d, %d, %d, %d"
                                     % (a, b, c, d, e, f, g, h))
            self.time_stamp_val.config(text="%.2f" % time.time())
            
            # Update average
            rssi_average_calc[0] += self.latest_serial_json['A']
            rssi_average_calc[1] += self.latest_serial_json['B']
            rssi_average_calc[2] += self.latest_serial_json['C']
            rssi_average_calc[3] += self.latest_serial_json['D']
            rssi_average_calc[4] += self.latest_serial_json['E']
            rssi_average_calc[5] += self.latest_serial_json['F']
            rssi_average_calc[6] += self.latest_serial_json['G']
            rssi_average_calc[7] += self.latest_serial_json['H']
            self.serial.reset_input_buffer()
            index += 1

            # Once the average is calculated update the array used in the positioning
            if index >= 5:
                rssi_average_calc /= 5
                index = 0
                self.rssi_average = np.copy(rssi_average_calc)
                rssi_average_calc.fill(0)
                
            time.sleep(0.2)

    """ Reads the latest datapoint from the MQTT client to be used
        in the Kalman filter. """
    def _mqtt_read(self):

        try:
            value = json.loads(self.mqttc.user_data_get()[0][0])['r1']

        except JSONDecodeError:
            value = -1
            print("JSON decode error")
        
        return value
 
    
    def _connect_mqtt(self, host, topic):

        mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        mqttc.on_connect = on_connect
        mqttc.on_message = on_message
        mqttc.on_subscribe = on_subscribe
        mqttc.on_unsubscribe = on_unsubscribe

        mqttc.user_data_set([0])
        mqttc.connect(host)
        mqttc.subscribe(topic)
        mqttc.loop_start()

        return mqttc
    
    
    def _canvas_data_map(self, x, y) -> tuple:

        new_x = ((x / MAX_LEN_ARENA) * GRAPH_SIZE) + GRAPH_OFFSET
        new_y = ((y / MAX_LEN_ARENA) * GRAPH_SIZE) + GRAPH_OFFSET
        
        return (new_x, new_y)

    def _update_canvas(self):

        # Clear the current data on the canvas.
        self._clear_graph()

        # # Get RSSI values
        # rssi_powers = self._json_to_list(self.latest_serial_json)
    
        # Get x, y position

        rssi_vals = position_from_rssi(self.rssi_average, [2,2], beacon_positions)

        self.k.predict()
        self.k.update(rssi_vals, self.meas_error_rssi)

        if self.mqtt_on:
            ultra_sonic = self._mqtt_read()
            if ultra_sonic > 0 and ultra_sonic < 400:
                self.k.update((2.0, ultra_sonic/100), self.meas_error_ultra * (ultra_sonic/100))
    
        *self.cur_pos, self.vx, self.vy = self.k.x_hat

        x, y = self._canvas_data_map(*self.cur_pos)
        x_stand, y_stand = self._canvas_data_map(*rssi_vals)

        # Filtered and RSSI averaged
        self._draw_point(x, y, 'red')

        # Unfiltered and RSSI averaged
        self._draw_point(x_stand, y_stand, 'blue')

        # Run this function every 0.5 seconds
        self.after(SCREEN_REFRESH, self._update_canvas)

    def _json_to_list(self, json_obj: json) -> list[int]:

        # print(json_obj)
        rssi_values = []
        rssi_values.append(json_obj['A'])
        rssi_values.append(json_obj['B'])
        rssi_values.append(json_obj['C'])
        rssi_values.append(json_obj['D'])
        rssi_values.append(json_obj['E'])
        rssi_values.append(json_obj['F'])
        rssi_values.append(json_obj['G'])
        rssi_values.append(json_obj['H'])
        
        return rssi_values


    def _canvas_setup(self):

        self.image = Image.open(Path.joinpath(Path(__file__).parent, 'images/grid_image.jpg')).resize((800, 800))
        self.grid = ImageTk.PhotoImage(self.image)
        self.graph_canvas = Canvas(master=self.display_graph, height=800, width=800)
        self.graph_canvas.grid(row=0, column=0)
        self.graph_canvas.create_image(0, 0, anchor='nw', image=self.grid)
        self._draw_beacon(25, 25, 15)
        self._draw_beacon(25, 775, 15)
        self._draw_beacon(775, 25, 15)
        self._draw_beacon(775, 775, 15)
        self._draw_beacon(400, 25, 15)
        self._draw_beacon(25, 400, 15)
        self._draw_beacon(400, 775, 15)
        self._draw_beacon(775, 400, 15)

    def _draw_beacon(self, x, y, width):

        points = [x-(width/2), y+width, 
                  x+(width/2), y+width, 
                  x+width, y, 
                  x+(width/2), y-width, 
                  x-(width/2), y-width, 
                  x-width, y]
     
        self.graph_canvas.create_polygon(points, fill='blue')

    

    def _add_button_event(self):

        def temp_text(e, text):
            text.delete(0,"end")

        display_window = Toplevel(self)
        display_window.geometry('430x350')
        display_window.title("Add Beacon")
        entries: list[Entry] = []

        for i, option in enumerate(OPTIONS):
    
            textbox = Entry(display_window, bg="white", width=50, borderwidth=2)
            textbox.grid(row=i, column=0, padx=5, pady=5, sticky='nsew')
            textbox.insert(0, option)
            entries.append(textbox)
            entries[i].grid(row=i, column=0, padx=5, pady=5, sticky='nsew')
            entries[i].bind("<FocusIn>", lambda e, text=textbox: temp_text(e, text))

        accept_button = Button(display_window, 
                               text="Add", 
                               command= lambda dw=display_window, entry_list=entries: self._add_button(dw, entry_list))
        accept_button.place(x=180, y=300)



    def _add_button(self, dw: Toplevel, entry_list: list[Entry]):

        name = entry_list[0].get()
        letter = name.split('-')[1].upper()
        self.valid_beacons.append(letter)
        addr = entry_list[1].get()
        major = entry_list[2].get()
        minor = entry_list[3].get()
        x = entry_list[4].get()
        y = entry_list[5].get()
        left = entry_list[6].get()
        right = entry_list[7].get()

        self.beacons[letter].x = x
        self.beacons[letter].y = y
        self.beacons[letter].left_neighbour = left
        self.beacons[letter].right_neighbour = right

        dw.destroy()


    def _remove_button_event(self):

        display_window = Toplevel(self)
        display_window.title("Remove Beacon")
        buttons: list[Button] = []
        for i, beacon in enumerate(self.valid_beacons):
            name = self.beacons[beacon].name
            button = Button(display_window, text=name, command= lambda beacon_name=name: self._remove_beacon(display_window, beacon_name))
            buttons.append(button)
            buttons[i].grid(row=i, column=0, padx=5, pady=5, sticky='nsew')

        

    def _remove_beacon(self, display_window: Toplevel, beacon_name: str = ''):

        keys = self.beacons.keys()
        for beacon in keys:
            name = self.beacons[beacon].name
            if name == beacon_name:
                self.valid_beacons.remove(beacon)

        display_window.destroy()


    def _display_beacon_info_event(self):

        display_window = Toplevel(self)
        display_window.title("Display Beacons")
        buttons: list[Button] = []
        buttons.append(Button(display_window, text='All', command= lambda: self._display_beacons(display_window)))
        buttons[0].grid(row=0, column=0, padx=5, pady=5, sticky='nsew')
        for i, beacon in enumerate(self.valid_beacons, start=1):
            name = self.beacons[beacon].name
            button = Button(display_window, text=name, command= lambda beacon_name=name: self._display_beacons(display_window, beacon_name))
            buttons.append(button)
            buttons[i].grid(row=i, column=0, padx=5, pady=5, sticky='nsew')


    def _connect_device(self, portName: str):

        ser = Serial()

        while True:
            print("Attempting to connect to %s ..." % portName)
            ports = serial.tools.list_ports.comports()  
            for port in ports:
                if port.device == portName:
                    ser = Serial(portName, baudrate=BAUDRATE)
                    print("Successfully connected to %s" % portName)
                    break

            if ser.is_open:
                return ser
            time.sleep(1)  
        

    def _clear_graph(self):

        for point in self.current_points:
            self.graph_canvas.delete(point)
        self.update()
        self.current_points = []

    

    def _display_beacons(self, display_window: Toplevel, beacon_name: str = ''):

        keys = self.beacons.keys()
        for beacon in keys:
            name = self.beacons[beacon].name
            if beacon_name == '' and beacon in self.valid_beacons:
                print(self.beacons[beacon])
            elif beacon_name == name:
                print(self.beacons[beacon])
                break

        display_window.destroy()


    def _update_pos(self):

        self.pos_label.config(text="Pos: %.2f, %.2f" % (self.cur_pos[0], self.cur_pos[1]))
        self.motion_label.config(text="Vx: %.2f Vy: %.2f" % (self.vx, self.vy))
        
        self.pos_label.after(SCREEN_REFRESH, self._update_pos)

    def run(self):

        # Set up serial read
        serial_thread = Thread(name='serial_thread', target=self._handle_serial)
        serial_thread.start()

        # Peform various updates every 500ms
        self.graph_canvas.after(SCREEN_REFRESH, self._update_canvas)
        self.pos_label.after(SCREEN_REFRESH, self._update_pos)
        self.mainloop()

       



        
    
        