#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 19:43:18 2022

@author: marco
"""

import networkx
import matplotlib.pyplot as plt
from networkx.drawing.nx_pydot import graphviz_layout



T_dict = {'root': {'M_n1': 1}, 'M_n1': {'L_n2': 1}, 'L_n2': {'L_n4': 1, 'R_n3': 1}, 'L_n4': {}, 'R_n3': {'L_n5': 1}, 'L_n5': {'L_n6': 1}, 'L_n6': {}} 

T_dict = {'root': {'L_n1': 1}, 'L_n1': {'L_n3': 1, 'R_n2': 1}, 'L_n3': {}, 'R_n2': {'L_n4': 1}, 'L_n4': {}}

T_dict = {'root': {'L_n1': 1, 'M_n2': 1}, 'L_n1': {}, 'M_n2': {'M_n4': 1, 'R_n3': 1}, 'M_n4': {'L_n5': 1}, 'L_n5': {}, 'R_n3': {}}

T_dict = {'root': {'M_n1': 1}, 'M_n1': {'M_n3': 1, 'R_n2': 1}, 'M_n3': {}, 'R_n2': {'R_n4': 1}, 'R_n4': {}}

T_dict = {'root': {'M_n1': 1}, 'M_n1': {'R_n2': 1}, 'R_n2': {'R_n3': 1}, 'R_n3': {'L_n4': 1}, 'L_n4': {}}

T_dict = {'root': {'M_n1': 1}, 'M_n1': {'L_n2': 1}, 'L_n2': {'L_n4': 1, 'R_n3': 1}, 'L_n4': {}, 'R_n3': {'L_n5': 1}, 'L_n5': {}}
T_dict = {'root': {'M_n1': 1}}
T_dict = {'root': {'L_n1': 1, 'M_n2': 1}, 'L_n1': {}, 'M_n2': {'M_n4': 1, 'R_n3': 1}, 'M_n4': {'L_n5': 1}, 'L_n5': {}, 'R_n3': {}}

T_dict = {'root': {'M_n1': 'M'}, 'M_n1': {'L_n2': 'L'}, 'L_n2': {'L_n4': 'L', 'R_n3': 'R'}, 'L_n4': {}, 'R_n3': {}}


T_dict = {'root': {'M_n1': 'M'}, 'M_n1': {'L_n2': 'L'}, 'L_n2': {'L_n4': 'L', 'R_n3': 'R'}, 'L_n4': {}, 'R_n3': {'L_n5': 'L'}, 'L_n5': {'L_n6': 'L'}, 'L_n6': {}} 

T_dict = {'root': {'M_n1': 'N'}, 'M_n1': {'L_n2': 'O'}, 'L_n2': {'L_n4': 'S', 'R_n3': 'N'}, 'L_n4': {}, 'R_n3': {'L_n5': 'O'}, 'L_n5': {'L_n6': 'S'}, 'L_n6': {}}
T_dict = {'root': {'M_n1': ' (M , N )'}, 'M_n1': {'L_n2': ' (L , O )'}, 'L_n2': {}}
T_dict = {'root': {'M_n1': ' (M , N )'}, 'M_n1': {'L_n2': ' (L , O )'}, 'L_n2': {'L_n4': ' (L , S )', 'R_n3': ' (R , N )'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O )'}, 'L_n5': {'L_n6': ' (L , S )'}, 'L_n6': {}} 
T_dict = {'root': {'M_n1': ' (M , N )'}, 'M_n1': {'L_n2': ' (L , O )', 'M_n3': ' (M , N )'}, 'L_n2': {}, 'M_n3': {'L_n4': ' (L , O )'}, 'L_n4': {'L_n5': ' (L , S )'}, 'L_n5': {}} 
T_dict = {'root': {'M_n2': ' (M , N )', 'R_n1': ' (R , E )'}, 'M_n2': {'R_n3': ' (R , E )'}, 'R_n3': {'L_n4': ' (L , N )'}, 'L_n4': {'L_n5': ' (L , O )'}, 'L_n5': {'R_n6': ' (R , N )'}, 'R_n6': {'R_n7': ' (R , E )'}, 'R_n7': {'L_n8': ' (L , N )'}, 'L_n8': {'L_n9': ' (L , O )'}, 'L_n9': {'R_n10': ' (R , N )'}, 'R_n10': {}, 'R_n1': {}} 

T_dict = {'root': {'M_n1': ' (M , N )'}, 'M_n1': {'L_n2': ' (L , O )'}, 'L_n2': {'L_n4': ' (L , S )', 'R_n3': ' (R , N )'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O )'}, 'L_n5': {'L_n6': ' (L , S )'}, 'L_n6': {'R_n7': ' (R , O )'}, 'R_n7': {'R_n8': ' (R , N )'}, 'R_n8': {'R_n9': ' (R , E )'}, 'R_n9': {'L_n10': ' (L , N )'}, 'L_n10': {'R_n11': ' (R , E )'}, 'R_n11': {'L_n13': ' (L , N )', 'R_n12': ' (R , S )'}, 'L_n13': {'L_n14': ' (L , O )'}, 'L_n14': {'L_n15': ' (L , S )'}, 'L_n15': {'R_n16': ' (R , O )'}, 'R_n16': {'L_n18': ' (L , S )', 'R_n17': ' (R , N )'}, 'L_n18': {}, 'R_n17': {'R_n19': ' (R , E )'}, 'R_n19': {'L_n20': ' (L , N )'}, 'L_n20': {'L_n21': ' (L , O )'}, 'L_n21': {}, 'R_n12': {}}

T_dict = {'root': {'M_n1': ' (M , N , Type.EXPLORED )'}, 'M_n1': {'L_n2': ' (L , O , Type.EXPLORED)'}, 'L_n2': {'L_n4': ' (L , S , Type.OBSERVED)', 'R_n3': ' (R , N , Type.EXPLORED)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , Type.EXPLORED)'}, 'L_n5': {'L_n6': ' (L , S , Type.OBSERVED)'}, 'L_n6': {}}

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , O)', 'R_n3': ' (R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , E)'}, 'L_n5': {'L_n6': ' (L , S , E)'}, 'L_n6': {'R_n7': ' (R , O , E)'}, 'R_n7': {'R_n8': ' (R , N , E)'}, 'R_n8': {'R_n9': ' (R , E , E)'}, 'R_n9': {'L_n10': ' (L , N , O)'}, 'L_n10': {}}

T_dict = {'root': {'L_n1': ' (L , O , E)', 'M_n2': ' (M , N , D )'}, 'L_n1': {'L_n3': ' (L , S , O)'}, 'L_n3': {}, 'M_n2': {}}

T_dict = {'root': {'L_n1': ' (L , O , E)', 'M_n2': ' (M , N , D )'}, 'L_n1': {'L_n3': ' (L , S , E)'}, 'L_n3': {'L_n4': ' (L , E , E)'}, 'L_n4': {'L_n5': ' (L , N , D)', 'M_n6': ' (M , E , E )'}, 'L_n5': {}, 'M_n6': {'L_n7': ' (L , N , E)'}, 'L_n7': {'M_n9': ' (M , N , E )', 'R_n8': ' (R , E , O)'}, 'M_n9': {'L_n10': ' (L , O , O)'}, 'L_n10': {}, 'R_n8': {}, 'M_n2': {}}

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , O)', 'R_n3': ' (R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , E)'}, 'L_n5': {'L_n6': ' (L , S , E)'}, 'L_n6': {'R_n7': ' (R , O , E)'}, 'R_n7': {'R_n8': ' (R , N , E)'}, 'R_n8': {'R_n9': ' (R , E , E)'}, 'R_n9': {'L_n10': ' (L , N , E)'}, 'L_n10': {'R_n11': ' (R , E , E)'}, 'R_n11': {'L_n13': ' (L , N , E)', 'R_n12': ' (R , S , O)'}, 'L_n13': {'L_n14': ' (L , O , E)'}, 'L_n14': {'L_n15': ' (L , S , E)'}, 'L_n15': {'R_n16': ' (R , O , E)'}, 'R_n16': {'L_n18': ' (L , S , O)', 'R_n17': ' (R , N , E)'}, 'L_n18': {}, 'R_n17': {'R_n19': ' (R , E , E)'}, 'R_n19': {'L_n20': ' (L , N , E)'}, 'L_n20': {'L_n21': ' (L , O , D)'}, 'L_n21': {}, 'R_n12': {}}

T_dict = {'root': {'M_n1': ' (M , O , E )'}, 'M_n1': {'L_n3': ' (L , S , O)', 'R_n2': ' (R , N , E)'}, 'L_n3': {}, 'R_n2': {'R_n4': ' (R , E , E)'}, 'R_n4': {'L_n5': ' (L , N , E)'}, 'L_n5': {'L_n6': ' (L , O , D)'}, 'L_n6': {}}

T_dict = {'root': {'M_n2': ' (M , E , D )', 'R_n1': ' (R , S , E)'}, 'M_n2': {'L_n3': ' (L , N , D)'}, 'L_n3': {}, 'R_n1': {'L_n4': ' (L , E , E)'}, 'L_n4': {'L_n5': ' (L , N , D)', 'M_n6': ' (M , E , E )'}, 'L_n5': {}, 'M_n6': {'L_n7': ' (L , N , O)'}, 'L_n7': {}}

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , D)', 'R_n3': ' (R , N , O)'}, 'L_n4': {}, 'R_n3': {}}

T_dict =  {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , D)', 'R_n3': ' (R , N , E)'}, 'L_n4': {'R_n5': ' (R , O , D)'}, 'R_n5': {'M_n7': ' (M , O , D )', 'R_n6': ' (R , N , D)'}, 'M_n7': {'R_n8': ' (R , N , D)'}, 'R_n8': {'R_n9': ' (R , E , D)'}, 'R_n9': {'L_n10': ' (L , N , D)'}, 'L_n10': {}, 'R_n6': {}, 'R_n3': {'L_n11': ' (L , O , E)'}, 'L_n11': {'L_n12': ' (L , S , E)'}, 'L_n12': {'R_n13': ' (R , O , E)'}, 'R_n13': {'R_n14': ' (R , N , E)'}, 'R_n14': {'R_n15': ' (R , E , E)'}, 'R_n15': {'L_n16': ' (L , N , E)'}, 'L_n16': {'R_n17': ' (R , E , E)'}, 'R_n17': {'L_n19': ' (L , N , E)', 'R_n18': ' (R , S , D)'}, 'L_n19': {'L_n20': ' (L , O , E)'}, 'L_n20': {'L_n21': ' (L , S , E)'}, 'L_n21': {'R_n22': ' (R , O , E)'}, 'R_n22': {'L_n24': ' (L , S , E)', 'R_n23': ' (R , N , O)'}, 'L_n24': {'R_n25': ' (R , O , E)'}, 'R_n25': {'L_n26': ' (L , S , E)'}, 'L_n26': {'R_n27': ' (R , O , E)'}, 'R_n27': {'R_n28': ' (R , N , E)'}, 'R_n28': {'R_n29': ' (R , E , E)'}, 'R_n29': {'L_n30': ' (L , N , E)'}, 'L_n30': {'L_n31': ' (L , O , E)'}, 'L_n31': {'R_n32': ' (R , N , E)'}, 'R_n32': {'R_n33': ' (R , E , E)'}, 'R_n33': {'L_n34': ' (L , N , E)'}, 'L_n34': {'L_n35': ' (L , O , O)'}, 'L_n35': {}, 'R_n23': {}, 'R_n18': {}}


T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , O)', 'R_n3': ' (R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , E)'}, 'L_n5': {'L_n6': ' (L , S , E)'}, 'L_n6': {'R_n7': ' (R , O , E)'}, 'R_n7': {'R_n8': ' (R , N , E)'}, 'R_n8': {'R_n9': ' (R , E , E)'}, 'R_n9': {'L_n10': ' (L , N , E)'}, 'L_n10': {'R_n11': ' (R , E , E)'}, 'R_n11': {'L_n13': ' (L , N , E)', 'R_n12': ' (R , S , O)'}, 'L_n13': {'L_n14': ' (L , O , E)'}, 'L_n14': {'L_n15': ' (L , S , E)'}, 'L_n15': {'R_n16': ' (R , O , E)'}, 'R_n16': {'L_n18': ' (L , S , E)', 'R_n17': ' (R , N , D)'}, 'L_n18': {'R_n22': ' (R , O , E)'}, 'R_n22': {'L_n23': ' (L , S , E)'}, 'L_n23': {'R_n24': ' (R , O , E)'}, 'R_n24': {'R_n25': ' (R , N , E)'}, 'R_n25': {'R_n26': ' (R , E , E)'}, 'R_n26': {'L_n27': ' (L , N , E)'}, 'L_n27': {'L_n28': ' (L , O , E)'}, 'L_n28': {'R_n29': ' (R , N , E)'}, 'R_n29': {'R_n30': ' (R , E , E)'}, 'R_n30': {'L_n31': ' (L , N , E)'}, 'L_n31': {'L_n32': ' (L , O , E)'}, 'L_n32': {'R_n33': ' (R , N , O)'}, 'R_n33': {}, 'R_n17': {'R_n19': ' (R , E , D)'}, 'R_n19': {'L_n20': ' (L , N , D)'}, 'L_n20': {'L_n21': ' (L , O , D)'}, 'L_n21': {}, 'R_n12': {}} 

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n3': ' (L , S , O)', 'M_n4': ' (M , O , E )'}, 'L_n3': {}, 'M_n4': {'M_n6': ' (M , O , O )', 'R_n5': ' (R , N , E)'}, 'M_n6': {}, 'R_n5': {'L_n7': ' (L , O , E)'}, 'L_n7': {'L_n9': ' (L , S , O)', 'R_n8': ' (R , N , E)'}, 'L_n9': {}, 'R_n8': {'L_n11': ' (L , O , E)', 'R_n10': ' (R , E , O)'}, 'L_n11': {'L_n13': ' (L , S , O)', 'R_n12': ' (R , N , E)'}, 'L_n13': {}, 'R_n12': {'R_n14': ' (R , E , E)'}, 'R_n14': {'L_n15': ' (L , N , E)'}, 'L_n15': {'L_n16': ' (L , O , E)', 'M_n17': ' (M , N , D )'}, 'L_n16': {'R_n29': ' (R , N , O)'}, 'R_n29': {}, 'M_n17': {'L_n19': ' (L , O , D)', 'R_n18': ' (R , E , D)'}, 'L_n19': {}, 'R_n18': {'M_n21': ' (M , E , D )', 'R_n20': ' (R , S , D)'}, 'M_n21': {'M_n23': ' (M , E , D )', 'R_n22': ' (R , S , D)'}, 'M_n23': {'R_n24': ' (R , S , D)'}, 'R_n24': {'R_n25': ' (R , O , D)'}, 'R_n25': {'L_n26': ' (L , S , D)'}, 'L_n26': {'L_n27': ' (L , E , D)'}, 'L_n27': {'L_n28': ' (L , N , D)'}, 'L_n28': {}, 'R_n22': {}, 'R_n20': {}, 'R_n10': {}}

T1 = networkx.DiGraph(T_dict)    
pos = graphviz_layout(T1, prog="dot")

plt.figure(figsize=(35,35))

networkx.draw(T1, pos, with_labels=True, node_size=450, font_size=8)

#labels_ = {('root', 'M_n1'): "1" }
labels_ = {}
for k,v in T_dict.items():
    for key in v:
        labels_[(k,key)] = str(v[key])
print(labels_)

networkx.draw_networkx_edge_labels(T1, pos, edge_labels=labels_)
# plt.savefig("Maze_Tree")
plt.show()











