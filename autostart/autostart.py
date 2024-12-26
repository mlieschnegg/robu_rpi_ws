from tkinter import *
from tkinter import ttk, messagebox
import threading
import time
import yaml
import os
from copy import deepcopy

from enum import Enum, auto

import signal
import sys

class LauncherGUI(Tk):
    
    def __init__(self):
        super().__init__()
        self.geometry("800x480")
        #root.attributes('-fullscreen', True)
        self.title("ROBU")

        self._pack_widgets()

    
    def _pack_widgets(self):
        # Grid festlegen
        Grid.rowconfigure(self,0,weight=1)
        Grid.columnconfigure(self,0,weight=1)

        Grid.rowconfigure(self,1,weight=1)

        frm_hostname = LabelFrame(self, text="Hostname")
        Grid.rowconfigure(frm_hostname,0,weight=1)
        Grid.columnconfigure(frm_hostname,0,weight=0)

        lbl = ttk.Label(frm_hostname, text="IP:")
        lbl.grid(row=0, column=0, sticky="NSEW")

        btn_quit = Button(self, text="Quit", command=self._btn_on_close)
        btn_quit.grid(row=1, column=0, sticky="NSEW")

    def _fill_widgets(self):
        try:
            pass
        except:
            pass

    def _btn_on_close(self):
        self.quit()

def launcher_gui():
    print("Test - GUI")
    gui = LauncherGUI()
    try:
        gui.mainloop()
    except KeyboardInterrupt:
        print('Sie haben Ctrl+C gedrückt!')
        gui.quit()

if __name__ == '__main__':
    launcher_gui()

    