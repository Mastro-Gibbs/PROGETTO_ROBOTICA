#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 19:43:18 2022

@author: marco
"""

import networkx
import matplotlib.pyplot as plt
from networkx.drawing.nx_pydot import graphviz_layout



T_dict = {'root': {'1': 1}, '1': {'2': 1}, '2': {'3': 1, '4': 1}}
T1 = networkx.DiGraph(T_dict)    
pos = graphviz_layout(T1, prog="dot")
networkx.draw(T1, pos, with_labels=True)
plt.show()