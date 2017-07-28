#!/usr/bin/env python

import Tkinter as tk
import threading
from enum import Enum
import exceptions
import sys
import time
import Queue
import rospy

class GUI(tk.Tk):

    class LogMode(Enum):
        NORMAL="normal"
        ERROR ="error"
        SUCCESS="success"
        PROCESSING="processing"
        WARN="warn"

    class stdoutRedirect():
        def __init__(self, text_output):
            self.text_output = text_output;
        def write(self, txt):
            self.text_output.log_mode(txt, App.LogMode.NORMAL,"std");

    class stderrRedirect():
        def __init__(self, text_output):
            self.text_output = text_output;
        def write(self, txt):
            self.text_output.log_mode(txt, App.LogMode.ERROR,"std")

    def __init__(self, master=None):
        tk.Tk.__init__(self,master);
        self.frame = tk.Frame(self);
        self.frames = {};
        self.buttons = {}; #dictionary of buttons.
        self.buttonStates = {};
        self.texts = {};   #dictionary of textboxes.
        self.scrollbars={};
        self.GridW = 0;
        self.GridH = 0
        self.output = None
        self.default_out = "Text"
        self.text = self.addText(self.default_out,50,20);
        self.protocol("WM_DELETE_WINDOW",self.quit);
        self.text.tag_configure("normal",foreground="white");
        self.text.tag_configure("error",foreground="red");
        self.text.tag_configure("success",foreground="green");
        self.text.tag_configure("processing",foreground="yellow");
        self.text.tag_configure("warn",foreground="orange");

        self.running = True;
        #sys.stdout = self.stdoutRedirect(self);
        #sys.stderr = self.stderrRedirect(self);
        self.frame.pack(side=tk.LEFT);


    def addButton(self, name, callback=None):
        button = tk.Button(self.frame,text=name, command=callback);
        button.pack();
        self.buttons[name] = button;

    def printButtons(self):
        app.log( self.buttons);

    def addText(self, name, w, h, bgcolor="black", fgcolor="white", side=tk.RIGHT, scroll=True):
        """adds a textbox to this gui.
        """
        frame = tk.Frame(self);
        self.frames[name]=frame;
        frame.pack(side=side);
        label = tk.Label(frame,text=name,anchor=tk.NW);
        label.pack(side=tk.TOP)
        text = tk.Text(frame, height=h,width=w, bg=bgcolor, fg=fgcolor);
        text.pack(side=tk.LEFT, expand=True);
        self.texts[name] = text;
        if scroll:
            scroll = tk.Scrollbar(frame)
            scroll.pack(side=tk.RIGHT,fill=tk.Y)
            scroll.config(command=text.yview)
            self.scrollbars[name] = scroll
        return text;

    def setOutput(self, textbox=""):
        if textbox == "":
            textbox = self.default_out;
        self.text = self.texts[textbox];

    def log_mode(self, text, logMode,src="App"):
        output = self.text;
        if not isinstance(text, basestring):
            to_write = src+" : " + str(text)+"\n";
            output.insert(tk.END, to_write,logMode.value);
        else:
            output.insert(tk.END, src+" : " +text+'\n',logMode.value);
        output.see(tk.END);

    def log(self, text):
        self.log_mode(text,self.LogMode.NORMAL);

    def error(self, text ):
        self.log_mode(text,self.LogMode.ERROR);

    def success(self, text ):
        self.log_mode(text,self.LogMode.SUCCESS);

    def progress(self, text ):
        self.log_mode(text,self.LogMode.PROCESSING);

    def warn(self, text ):
        self.log_mode(text,self.LogMode.WARN);

    def logFromQueue(self, queue, mode = LogMode.NORMAL, output=""):
        """Gets all elements from queue and logs it with mode onto output"""
        self.setOutput(output);
        while not queue.empty():
            txt = queue.get()
            self.log_mode(txt,mode,output)
            queue.task_done();

    def quit(self):
        self.running =False;

class PrintQueue():
    """sys.stdout = PrintQueue(queue)
    to redirect print from console to the given queue
    """
    def __init__(self, q):
        self.q = q;

    def write(self,text):
        self.q.put(text);



if __name__ == '__main__':

    rospy.init_node("wkkw")

    import threading;
    gui = GUI();
    q1 = Queue.Queue();
    q2 = Queue.Queue();

    oldout = sys.stdout;
    sys.stdout = PrintQueue(q1);
    sys.stderr = PrintQueue(q2);

    def sayhello():
        with threading.Lock():
            q1.put("Sathello");


    def toggle():
        with threading.Lock():
            q2.put("Toggle")
            t = threading.Thread(target=threaded);
            t.start();

    def threaded():
        with threading.Lock():
            for i in range(0,100,1):
                if gui.running == False:
                    break;
                q2.put("ggdg")
                rospy.loginfo("ddj")
                rospy.logerr("Hello")

    gui.addButton("hello",sayhello)
    gui.addButton("bye",toggle)
    gui.addText("Debug",50,20);

    i=0;
    while gui.running:
        gui.update();
        gui.logFromQueue(q2,output="Debug")
        gui.logFromQueue(q1,mode = GUI.LogMode.WARN)


    sys.stdout = oldout
    gui.destroy();