"""
This program is used to draw graphically the Maze tree using networkx and graphviz.
The tree is saved as a dict type where the key is the node and the value is another dict composed by
node's children. Each child of the node has, as value, a string of three values:
    1) Left/Mid/Right
    2) NORD/EST/OVEST/SUD
    3) EXPLORED/OBSERVED/DEAD_END/FINAL
"""

import networkx
import matplotlib.pyplot as plt
from networkx.drawing.nx_pydot import graphviz_layout


T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , O)', 'R_n3': ' (R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , E)'}, 'L_n5': {'L_n6': ' (L , S , E)'}, 'L_n6': {'R_n7': ' (R , O , E)'}, 'R_n7': {'R_n8': ' (R , N , E)'}, 'R_n8': {'R_n9': ' (R , E , E)'}, 'R_n9': {'L_n10': ' (L , N , E)'}, 'L_n10': {'R_n11': ' (R , E , E)'}, 'R_n11': {'L_n13': ' (L , N , E)', 'R_n12': ' (R , S , O)'}, 'L_n13': {'L_n14': ' (L , O , E)'}, 'L_n14': {'L_n15': ' (L , S , E)'}, 'L_n15': {'R_n16': ' (R , O , E)'}, 'R_n16': {'L_n18': ' (L , S , E)', 'R_n17': ' (R , N , D)'}, 'L_n18': {'R_n22': ' (R , O , E)'}, 'R_n22': {'L_n23': ' (L , S , E)'}, 'L_n23': {'R_n24': ' (R , O , E)'}, 'R_n24': {'R_n25': ' (R , N , E)'}, 'R_n25': {'R_n26': ' (R , E , E)'}, 'R_n26': {'L_n27': ' (L , N , E)'}, 'L_n27': {'L_n28': ' (L , O , E)'}, 'L_n28': {'R_n29': ' (R , N , E)'}, 'R_n29': {'R_n30': ' (R , E , E)'}, 'R_n30': {'L_n31': ' (L , N , E)'}, 'L_n31': {'L_n32': ' (L , O , E)'}, 'L_n32': {'R_n33': ' (R , N , O)'}, 'R_n33': {}, 'R_n17': {'R_n19': ' (R , E , D)'}, 'R_n19': {'L_n20': ' (L , N , D)'}, 'L_n20': {'L_n21': ' (L , O , D)'}, 'L_n21': {}, 'R_n12': {}} 

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n3': ' (L , S , O)', 'M_n4': ' (M , O , E )'}, 'L_n3': {}, 'M_n4': {'M_n6': ' (M , O , O )', 'R_n5': ' (R , N , E)'}, 'M_n6': {}, 'R_n5': {'L_n7': ' (L , O , E)'}, 'L_n7': {'L_n9': ' (L , S , O)', 'R_n8': ' (R , N , E)'}, 'L_n9': {}, 'R_n8': {'L_n11': ' (L , O , E)', 'R_n10': ' (R , E , O)'}, 'L_n11': {'L_n13': ' (L , S , O)', 'R_n12': ' (R , N , E)'}, 'L_n13': {}, 'R_n12': {'R_n14': ' (R , E , E)'}, 'R_n14': {'L_n15': ' (L , N , E)'}, 'L_n15': {'L_n16': ' (L , O , E)', 'M_n17': ' (M , N , D )'}, 'L_n16': {'R_n29': ' (R , N , O)'}, 'R_n29': {}, 'M_n17': {'L_n19': ' (L , O , D)', 'R_n18': ' (R , E , D)'}, 'L_n19': {}, 'R_n18': {'M_n21': ' (M , E , D )', 'R_n20': ' (R , S , D)'}, 'M_n21': {'M_n23': ' (M , E , D )', 'R_n22': ' (R , S , D)'}, 'M_n23': {'R_n24': ' (R , S , D)'}, 'R_n24': {'R_n25': ' (R , O , D)'}, 'R_n25': {'L_n26': ' (L , S , D)'}, 'L_n26': {'L_n27': ' (L , E , D)'}, 'L_n27': {'L_n28': ' (L , N , D)'}, 'L_n28': {}, 'R_n22': {}, 'R_n20': {}, 'R_n10': {}}

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , O)'}, 'L_n2': {}}

T_dict = {'root': {'M_n1': ' (M , N , E )'}, 'M_n1': {'L_n2': ' (L , O , E)'}, 'L_n2': {'L_n4': ' (L , S , O)', 'R_n3': ' (R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': ' (L , O , E)'}, 'L_n5': {'L_n6': ' (L , S , E)'}, 'L_n6': {'R_n7': ' (R , O , E)'}, 'R_n7': {'R_n8': ' (R , N , E)'}, 'R_n8': {'R_n9': ' (R , E , E)'}, 'R_n9': {'L_n10': ' (L , N , E)'}, 'L_n10': {'R_n11': ' (R , E , E)'}, 'R_n11': {'L_n13': ' (L , N , E)', 'R_n12': ' (R , S , O)'}, 'L_n13': {'L_n14': ' (L , O , E)'}, 'L_n14': {'L_n15': ' (L , S , E)'}, 'L_n15': {'R_n16': ' (R , O , E)'}, 'R_n16': {'L_n18': ' (L , S , E)', 'R_n17': ' (R , N , D)'}, 'L_n18': {'R_n22': ' (R , O , E)'}, 'R_n22': {'L_n23': ' (L , S , E)'}, 'L_n23': {'R_n24': ' (R , O , E)'}, 'R_n24': {'R_n25': ' (R , N , E)'}, 'R_n25': {'R_n26': ' (R , E , E)'}, 'R_n26': {'L_n27': ' (L , N , E)'}, 'L_n27': {'L_n28': ' (L , O , E)'}, 'L_n28': {'R_n29': ' (R , N , E)'}, 'R_n29': {'R_n30': ' (R , E , E)'}, 'R_n30': {'L_n31': ' (L , N , E)'}, 'L_n31': {'L_n32': ' (L , O , E)'}, 'L_n32': {'R_n33': ' (R , N , O)'}, 'R_n33': {}, 'R_n17': {'R_n19': ' (R , E , D)'}, 'R_n19': {'L_n20': ' (L , N , D)'}, 'L_n20': {'L_n21': ' (L , O , D)'}, 'L_n21': {}, 'R_n12': {}}

T_dict = {'root': {'R_n1': ' (R , E , E)'}, 'R_n1': {'L_n2': ' (L , N , E)'}, 'L_n2': {'L_n3': ' (L , O , F)'}, 'L_n3': {}} 

T_dict = {'root': {'L_n1': '(L , O , E)', 'M_n2': '(M , N , D)'}, 'L_n1': {'L_n3': '(L , S , E)'}, 'L_n3': {'L_n4': '(L , E , E)'}, 'L_n4': {'L_n5': '(L , N , D)', 'M_n6': '(M , E , E)'}, 'L_n5': {}, 'M_n6': {'L_n7': '(L , N , E)'}, 'L_n7': {'M_n9': '(M , N , O)', 'R_n8': '(R , E , O)'}, 'M_n9': {}, 'R_n8': {}, 'M_n2': {}} 


T_dict = {'root': {'M_n1': '(M , N , E)'}, 'M_n1': {'L_n2': '(L , O , E)'}, 'L_n2': {'L_n4': '(L , S , O)', 'R_n3': '(R , N , E)'}, 'L_n4': {}, 'R_n3': {'L_n5': '(L , O , E)'}, 'L_n5': {'L_n6': '(L , S , E)'}, 'L_n6': {'R_n7': '(R , O , E)'}, 'R_n7': {'R_n8': '(R , N , E)'}, 'R_n8': {'R_n9': '(R , E , E)'}, 'R_n9': {'L_n10': '(L , N , E)'}, 'L_n10': {'R_n11': '(R , E , E)'}, 'R_n11': {'L_n13': '(L , N , E)', 'R_n12': '(R , S , O)'}, 'L_n13': {'L_n14': '(L , O , E)'}, 'L_n14': {'L_n15': '(L , S , E)'}, 'L_n15': {'R_n16': '(R , O , E)'}, 'R_n16': {'L_n18': '(L , S , E)', 'R_n17': '(R , N , D)'}, 'L_n18': {'R_n22': '(R , O , E)'}, 'R_n22': {'L_n23': '(L , S , E)'}, 'L_n23': {'R_n24': '(R , O , E)'}, 'R_n24': {'R_n25': '(R , N , E)'}, 'R_n25': {'R_n26': '(R , E , E)'}, 'R_n26': {'L_n27': '(L , N , E)'}, 'L_n27': {'L_n28': '(L , O , E)'}, 'L_n28': {'R_n29': '(R , N , E)'}, 'R_n29': {'R_n30': '(R , E , E)'}, 'R_n30': {'L_n31': '(L , N , E)'}, 'L_n31': {'L_n32': '(L , O , F)'}, 'L_n32': {}, 'R_n17': {'R_n19': '(R , E , D)'}, 'R_n19': {'L_n20': '(L , N , D)'}, 'L_n20': {'L_n21': '(L , O , D)'}, 'L_n21': {}, 'R_n12': {}} 

T_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'L_n3': '(L , O , E)'}, 'L_n3': {'L_n5': '(L , S , O)', 'R_n4': '(R , N , E)'}, 'L_n5': {}, 'R_n4': {'L_n6': '(L , O , E)'}, 'L_n6': {'L_n7': '(L , S , E)'}, 'L_n7': {'R_n8': '(R , O , E)'}, 'R_n8': {'R_n9': '(R , N , E)'}, 'R_n9': {'R_n10': '(R , E , E)'}, 'R_n10': {'L_n11': '(L , N , E)'}, 'L_n11': {'R_n12': '(R , E , E)'}, 'R_n12': {'L_n14': '(L , N , E)', 'R_n13': '(R , S , O)'}, 'L_n14': {'L_n15': '(L , O , E)'}, 'L_n15': {'L_n16': '(L , S , E)'}, 'L_n16': {'R_n17': '(R , O , E)'}, 'R_n17': {'L_n19': '(L , S , E)', 'R_n18': '(R , N , D)'}, 'L_n19': {'R_n23': '(R , O , E)'}, 'R_n23': {'L_n24': '(L , S , E)'}, 'L_n24': {'R_n25': '(R , O , E)'}, 'R_n25': {'R_n26': '(R , N , E)'}, 'R_n26': {'R_n27': '(R , E , E)'}, 'R_n27': {'L_n28': '(L , N , E)'}, 'L_n28': {'L_n29': '(L , O , E)'}, 'L_n29': {'R_n30': '(R , N , E)'}, 'R_n30': {'R_n31': '(R , E , E)'}, 'R_n31': {'L_n32': '(L , N , E)'}, 'L_n32': {'L_n33': '(L , O , E)'}, 'L_n33': {'R_n34': '(R , N , F)'}, 'R_n34': {}, 'R_n18': {'R_n20': '(R , E , D)'}, 'R_n20': {'L_n21': '(L , N , D)'}, 'L_n21': {'L_n22': '(L , O , D)'}, 'L_n22': {}, 'R_n13': {}}

T1 = networkx.DiGraph(T_dict)    
pos = graphviz_layout(T1, prog="dot")

plt.figure(figsize=(35, 35))

networkx.draw(T1, pos, with_labels=True, node_size=450, font_size=8)

labels_ = {}
for k, v in T_dict.items():
    for key in v:
        labels_[(k, key)] = str(v[key])

networkx.draw_networkx_edge_labels(T1, pos, edge_labels=labels_)
# plt.savefig("Maze_Tree")
plt.show()











