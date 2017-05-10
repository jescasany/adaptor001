#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 10:57:25 2017

@author: juan
"""

# to translate from 'e#r#' format to clear 'meaning'
class Decode:
    def __init__(self, raw):
        #pdb.set_trace()
        self.raw = raw
    
    def get_translation(self):
        #pdb.set_trace()
        self.raw = self.raw.replace('e1r10','move forward fail')
        self.raw = self.raw.replace('e1r1','move forward wall')
        self.raw = self.raw.replace('e1r4','move forward no wall')
        self.raw = self.raw.replace('e2r2','turn left')
        self.raw = self.raw.replace('e3r3','turn right')
        self.raw = self.raw.replace('e4r4','front free')
        self.raw = self.raw.replace('e4r5','front busy')
        self.raw = self.raw.replace('e5r6','right1 sensing')
        self.raw = self.raw.replace('e5r8','right2 sensing')
        self.raw = self.raw.replace('e5r12','right3 sensing')
        self.raw = self.raw.replace('e5r14','nothing on right1')
        self.raw = self.raw.replace('e6r7','left1 sensing')
        self.raw = self.raw.replace('e6r9','left2 sensing')
        self.raw = self.raw.replace('e6r11','left3 sensing')
        self.raw = self.raw.replace('e6r13','nothing on left1')
        self.raw = self.raw.replace('e1','move forward')
        self.raw = self.raw.replace('e2','turn left')
        self.raw = self.raw.replace('e3','turn right')
        self.raw = self.raw.replace('e4','front?')
        self.raw = self.raw.replace('e5','right?')
        self.raw = self.raw.replace('e6','left?')
        return self.raw
