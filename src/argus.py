#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 22 00:19:56 2017

@author: juan

You have to install tmux before
"""

import sys, time
from PySide.QtCore import *
from PySide.QtGui import *

class embeddedTerminal(QWidget):

    def __init__(self):
        QWidget.__init__(self)
        self.mechanism = ''
        self._processes = []
        self.resize(600, 600)
        self.terminal = QWidget(self)
        self.terminal2 = QWidget(self)
        layout = QGridLayout(self)
        
        button = QPushButton('Start')
        layout.addWidget(button, 0, 0)
        button.clicked.connect(self._start)
        button2 = QPushButton('Pause')
        layout.addWidget(button2, 0, 1)
        button2.clicked.connect(self._pause)
        button3 = QPushButton('Stop')
        layout.addWidget(button3, 0, 2)
        button3.clicked.connect(self._stop)
        button4 = QPushButton('Close')
        layout.addWidget(button4, 0, 3)
        button4.clicked.connect(self._close)
        button5 = QPushButton('Quit')
        layout.addWidget(button5, 0, 4)
        button5.clicked.connect(self._quit)
        button6 = QPushButton('Environment')
        layout.addWidget(button6, 2, 5)
        button6.clicked.connect(self._environment)
        button7 = QPushButton('Stop Environment')
        layout.addWidget(button7, 3, 5)
        button7.clicked.connect(self._stopenvironment)
        button8 = QPushButton('Close Environment')
        layout.addWidget(button8, 4, 5)
        button8.clicked.connect(self._closeenvironment)
        
        radio_button = QRadioButton('Simple')
        layout.addWidget(radio_button, 10, 5)
        radio_button.clicked.connect(self._simple) 
        radio_button2 = QRadioButton('Recursive')
        layout.addWidget(radio_button2, 11, 5)
        radio_button2.clicked.connect(self._recursive)
        radio_button3 = QRadioButton('Constructive')
        layout.addWidget(radio_button3, 12, 5)
        radio_button3.clicked.connect(self._constructive)
        
        layout.addWidget(self.terminal2, 1, 0, 8, 5)
        self._start_process('xterm', ['-into', str(self.terminal2.winId()), '-e', 'tmux', 'new', '-s', 'my_session2'])
        layout.addWidget(self.terminal, 9, 0, 16, 5)
        self._start_process('xterm', ['-into', str(self.terminal.winId()), '-e', 'tmux', 'new', '-s', 'my_session'])

    def _start_process(self, prog, args):
        child = QProcess()
        self._processes.append(child)
        child.start(prog, args)

    def _start(self):
        try:
            self._start_process('tmux', ['send-keys', '-t', 'my_session:0', self.mechanism, 'Enter'])
        except Exception:
            print('Please, choose a mechanism.', sys.exc_info()[1])
            
    def _pause(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session:0', 'ls', 'Enter'])
        
    def _stop(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session:0', '^C', 'Enter'])
        
    def _close(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session:0', 'exit', 'Enter'])
        
    def _quit(self):
        sys.exit(0)
        
    def _environment(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session2:0', 'roslaunch adaptor001 vel_stage.launch map_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.yaml" world_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.world" initial_pose_x:=18.0 initial_pose_y:=-4.9 initial_pose_a:=3.141592', 'Enter'])
        
    def _stopenvironment(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session2:0', '^C', 'Enter'])
        
    def _closeenvironment(self):
        self._start_process('tmux', ['send-keys', '-t', 'my_session2:0', 'exit', 'Enter'])
        
    def _simple(self):
        self.mechanism = 'rosrun adaptor001 eca_agent05.py simple'
        
    def _recursive(self):
        self.mechanism = 'rosrun adaptor001 eca_agent05.py recursive'
        
    def _constructive(self):
        self.mechanism = 'rosrun adaptor001 eca_agent05.py constructive'

if __name__ == "__main__":
    # Exception Handling
    try:
        app = QApplication(sys.argv)
        main = embeddedTerminal()
        main.show()
        sys.exit(app.exec_())
    except NameError:
        print("Name Error:", sys.exc_info()[1])
    except SystemExit:
        print("Closing Window...")
    except Exception:
        print(sys.exc_info()[1])     