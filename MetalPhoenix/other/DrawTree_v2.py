"""
Version 2 - no pydot
This program is used to draw graphically the Maze tree using networkx and graphviz.
The tree is saved as a dict type where the key is the node and the value is another dict composed by
node's children. Each child of the node has, as value, a string of three values:
    1) Left/Mid/Right
    2) NORD/EST/OVEST/SUD
    3) EXPLORED/OBSERVED/DEAD_END/FINAL
"""

import networkx as nx
import matplotlib.pyplot as plt
import configparser
import os
import ast

config = configparser.ConfigParser()
path = "../robot/resources/data/"
file_name = "data_analysis.conf"
conf_file = path + file_name

if os.path.isfile(conf_file):
    print(f"\nFile {file_name} loaded")
    config.read(conf_file)
else:
    print("Error: file " + conf_file + " not found")
    SystemError()


i = 1

# Print the trees of the all sections of the configuration file "data_analysis.conf"
for section in config.sections():
    
    print("Plotting " + str(i) + "°" + " tree...")
    print("Info: " + section)
    tree_dict = config[section]["tree_dict"]
    tree_dict = ast.literal_eval(tree_dict)
    
    T1 = nx.DiGraph(tree_dict)
    pos = nx.nx_agraph.graphviz_layout(T1, prog="dot")
    plt.figure(figsize=(35, 35))
    nx.draw(T1, pos, with_labels=True, node_size=450, font_size=8)

    labels_ = {}
    for k, v in tree_dict.items():
        for key in v:
            labels_[(k, key)] = str(v[key])

    nx.draw_networkx_edge_labels(T1, pos, edge_labels=labels_)
    plt.savefig("Tree_" + section)
    plt.show()
    i = i + 1

"""

# Single plot

# tree_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'L_n3': '(L , O , E)'}, 'L_n3': {'L_n5': '(L , S , O)', 'R_n4': '(R , N , E)'}, 'L_n5': {}, 'R_n4': {'L_n6': '(L , O , E)'}, 'L_n6': {'L_n7': '(L , S , E)'}, 'L_n7': {'R_n8': '(R , O , E)'}, 'R_n8': {'R_n9': '(R , N , E)'}, 'R_n9': {'R_n10': '(R , E , E)'}, 'R_n10': {'L_n11': '(L , N , E)'}, 'L_n11': {'R_n12': '(R , E , E)'}, 'R_n12': {'L_n14': '(L , N , E)', 'R_n13': '(R , S , O)'}, 'L_n14': {'L_n15': '(L , O , E)'}, 'L_n15': {'L_n16': '(L , S , E)'}, 'L_n16': {'R_n17': '(R , O , E)'}, 'R_n17': {'L_n19': '(L , S , E)', 'R_n18': '(R , N , D)'}, 'L_n19': {'R_n23': '(R , O , E)'}, 'R_n23': {'L_n24': '(L , S , E)'}, 'L_n24': {'R_n25': '(R , O , E)'}, 'R_n25': {'R_n26': '(R , N , E)'}, 'R_n26': {'R_n27': '(R , E , E)'}, 'R_n27': {'L_n28': '(L , N , E)'}, 'L_n28': {'L_n29': '(L , O , E)'}, 'L_n29': {'R_n30': '(R , N , E)'}, 'R_n30': {'R_n31': '(R , E , E)'}, 'R_n31': {'L_n32': '(L , N , E)'}, 'L_n32': {'L_n33': '(L , O , F)'}, 'L_n33': {}, 'R_n18': {'R_n20': '(R , E , D)'}, 'R_n20': {'L_n21': '(L , N , D)'}, 'L_n21': {'L_n22': '(L , O , D)'}, 'L_n22': {}, 'R_n13': {}}
# tree_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'M_n3': '(M , N , E)'}, 'M_n3': {'L_n5': '(L , O , E)', 'M_n6': '(M , N , D)', 'R_n4': '(R , E , D)'}, 'L_n5': {'L_n8': '(L , S , E)'}, 'L_n8': {'L_n10': '(L , E , O)', 'M_n11': '(M , S , O)', 'R_n9': '(R , O , O)'}, 'L_n10': {}, 'M_n11': {}, 'R_n9': {}, 'M_n6': {'L_n7': '(L , O , D)'}, 'L_n7': {}, 'R_n4': {}}

# BUG:
# tree_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'L_n4': '(L , W , E)', 'M_n5': '(M , N , D)', 'R_n3': '(R , E , D)'}, 'L_n4': {'L_n7': '(L , S , O)', 'M_n8': '(M , W , E)'}, 'L_n7': {}, 'M_n8': {'L_n9': '(L , S , E)'}, 'L_n9': {'L_n11': '(L , E , O)', 'M_n12': '(M , S , O)', 'R_n10': '(R , W , O)'}, 'L_n11': {}, 'M_n12': {}, 'R_n10': {}, 'M_n5': {'L_n6': '(L , W , D)'}, 'L_n6': {}, 'R_n3': {}}
# BUG:
# tree_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'R_n3': '(R , E , E)'}, 'R_n3': {'M_n5': '(M , E , D)', 'R_n4': '(R , S , E)'}, 'M_n5': {}, 'R_n4': {'L_n7': '(L , E , O)', 'M_n8': '(M , S , O)', 'R_n6': '(R , W , O)'}, 'L_n7': {}, 'M_n8': {}, 'R_n6': {}}

# tree_dict = {'root': {'M_n2': '(M , N , E)'}, 'M_n2': {'L_n3': '(L , W , E)'}, 'L_n3': {'L_n5': '(L , S , O)', 'R_n4': '(R , N , E)'}, 'L_n5': {}, 'R_n4': {'L_n6': '(L , W , E)'}, 'L_n6': {'L_n7': '(L , S , E)'}, 'L_n7': {'R_n8': '(R , W , E)'}, 'R_n8': {'R_n9': '(R , N , E)'}, 'R_n9': {'R_n10': '(R , E , E)'}, 'R_n10': {'L_n11': '(L , N , E)'}, 'L_n11': {'R_n12': '(R , E , E)'}, 'R_n12': {'L_n14': '(L , N , E)', 'R_n13': '(R , S , O)'}, 'L_n14': {'L_n15': '(L , W , E)'}, 'L_n15': {'L_n16': '(L , S , E)'}, 'L_n16': {'R_n17': '(R , W , E)'}, 'R_n17': {'L_n19': '(L , S , E)', 'R_n18': '(R , N , D)'}, 'L_n19': {'R_n23': '(R , W , E)'}, 'R_n23': {'L_n24': '(L , S , E)'}, 'L_n24': {'R_n25': '(R , W , E)'}, 'R_n25': {'R_n26': '(R , N , E)'}, 'R_n26': {'R_n27': '(R , E , E)'}, 'R_n27': {'L_n28': '(L , N , E)'}, 'L_n28': {'L_n29': '(L , W , E)'}, 'L_n29': {'R_n30': '(R , N , E)'}, 'R_n30': {'R_n31': '(R , E , E)'}, 'R_n31': {'L_n32': '(L , N , E)'}, 'L_n32': {'L_n33': '(L , W , E)'}, 'L_n33': {'FINAL': '(R , N , F)'}, 'FINAL': {}, 'R_n18': {'R_n20': '(R , E , D)'}, 'R_n20': {'L_n21': '(L , N , D)'}, 'L_n21': {'L_n22': '(L , W , D)'}, 'L_n22': {}, 'R_n13': {}}

T1 = nx.DiGraph(tree_dict)
pos = nx.nx_agraph.graphviz_layout(T1, prog="dot")
plt.figure(figsize=(35, 35))
nx.draw(T1, pos, with_labels=True, node_size=450, font_size=8)

labels_ = {}
for k, v in tree_dict.items():
    for key in v:
        labels_[(k, key)] = str(v[key])

nx.draw_networkx_edge_labels(T1, pos, edge_labels=labels_)
"""
