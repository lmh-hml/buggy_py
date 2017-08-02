#!/usr/bin/env python

import Tkinter as tk
import tkFileDialog as tkfile
import threading
from enum import Enum
import exceptions
import sys
import time
import Queue




if __name__ == '__main__':


    top = tk.Tk();
    buttons = [];
    f1 = tk.Frame(top,padx = 0,bg="#ff0000");
    f1.grid();

    txt = "1234567890"
    tl = len(txt)
    for i in range(0,5):
        b =  tk.Button(f1,text ="1234567890",height =1, width=tl,padx = 0);
        buttons.append(b)
        b.grid(row=0,column=i)


    t1 = tk.Text(top, height=20);
    t1.grid()

    t2 = tk.Text(top,  height=20);
    t2.grid();

    print top.grid_size()
    print f1.grid_size()

    top.mainloop()
