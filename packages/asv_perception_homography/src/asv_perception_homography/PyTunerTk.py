"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

# Tkinter-based python tuner
#import tkinter as tk  # python 3.x
import Tkinter as tk
import tkMessageBox as messagebox

#create window & frames
class Tuner:
    # ctlDefs array; array of array-like control definitions
    #   type: entry (single-line text input)
    #       ('entry', label, default, command )
    #   type: slider
    #       ('slider', label, default, min, max, resolution, command )
    #   type: button
    #       ('button', label, command )
    def __init__(self, ctlDefs, title='PyTunerTk' ):
        self.root = tk.Tk()
        self.root.title(title)
        self.frame = tk.Frame(self.root, width=500, takefocus=1 )
        self.frame.pack(fill=tk.BOTH, expand=1)
        self.values = dict()
        
        i = 0
        for def_ in ctlDefs:
            ctl = None  # the tk control being created
            lbl = None  # row label

            if def_[0] == 'slider':
                ctl = tk.Scale(self.frame, 
                               from_=def_[3], to=def_[4], resolution=def_[5],
                               digits=5, orient="horizontal", takefocus=1,length=300,
                               command=lambda val, name=def_[1], cb=def_[6] : self.on_value_change( name, val, cb )
                    )
                ctl.set( def_[2] ) # set initial value
                lbl = def_[1]  # set row label value
                self.values[lbl]=def_[2]  # store initial value in dict

            elif def_[0]=='button':
                ctl = tk.Button(self.frame, text=def_[1], command=lambda cb=def_[2] : cb(self.values) )
            elif def_[0]=='entry':
                ctl = tk.Entry( self.frame, width=30 )
                ctl.configure(validate='focusout'
                    , vcmd=lambda name=def_[1], cb=def_[3], ctl=ctl : self.on_value_change( name, ctl.get(), cb, 'string' ) 
                )
                ctl.bind("<Return>", (lambda event: self.frame.focus() ))
                ctl.insert( 0, def_[2] )
                lbl = def_[1]
                self.values[lbl]=def_[2]  # store initial value in dict
            else:
                raise NotImplementedError(def_[0])

            if not lbl is None: # we want a row label
                tk.Label(self.frame, text=lbl).grid(row=i,column=0)

            # add ctl to grid
            ctl.grid(row=i,column=1)
            i += 1

    def on_value_change(self, name, value, cb, dtype='float' ):
        
        if value is not None:
            v = value
            if dtype=='float':
                v = float(v)
            elif dtype=='int':
                v = int(v)
            self.values[name]= v

        if cb is not None:
            cb( self.values )

        return True  # for entry validate

    def show(self):
        self.root.mainloop()

    def close( self ):
        self.root.destroy()

def show_info_messagebox( msg ):
    messagebox.showinfo(message=msg)

def show_yesno_messagebox( msg ):
    return messagebox.askyesno(message=msg)