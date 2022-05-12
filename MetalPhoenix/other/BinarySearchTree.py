#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 12:57:53 2022

@author: marco
"""

"""
Binary Search tree (senza bilanciamento):
1) Ogni nodo v contiene un elemento elem(v) cui è associata una chiave chiave(v)
    presa da un dominio totalmente ordinato
2) Le chiavi del sottoalbero sx di v sono < chiave(v)
3) Le chiavi del sottoalbero dx di v sono > chiave(v)
""" 

import time
import random

import networkx
import matplotlib.pyplot as plt
import pydot
from networkx.drawing.nx_pydot import graphviz_layout


class Node:
    def __init__(self, key, element = "Node"):
        self.key = key
        self.element = element
        self.leftchild = None
        self.rightchild = None
        self.parent = None
    
    def isLeaf(self):
        return self.get_leftchild() is None and self.get_rightchild() is None
    
    def set_parent(self, node):
        self.parent = node
    
    def set_leftchild(self, node):
        self.leftchild = node
    
    def set_rightchild(self, node):
        self.rightchild = node
    
    def set_element(self, elem):
        self.element = elem
    
    def get_parent(self):
        return self.parent
    
    def get_leftchild(self):
        return self.leftchild
    
    def get_rightchild(self):
        return self.rightchild
    
    def get_key(self):
        return self.key
    
    def get_element(self):
        return self.element
    
    def __str__(self):
        return "Node (k,elem): ({},{})".format(self.get_key(), self.get_element())
    
class BST:
    def __init__(self, data):
        self.datakeys = data.copy()
        self.root = None
        while self.datakeys:
            key = self.datakeys.pop()
            self.insert(key)
        self.T = {self.root.get_key(): {}}
        
    def insert(self, key):
        node = Node(key)
        if self.root is None:
            node.set_element("Root")
            self.root = node
        else:
            parent = self.searchParentLeaf(key)
            if parent.get_key() > key:
                parent.set_leftchild(node)
                node.set_parent(parent)
            elif parent.get_key() < node.get_key():
                parent.set_rightchild(node)
                node.set_parent(parent)
                
    # Search parent usato per fare la insert di un nuovo elemento
    # come figlio che diventa foglia
    def searchParentLeaf(self, key):
        v = self.root
        if v.get_key() == key:
            return None
        while v is not None:
            if v.get_key() > key:           
                if v.get_leftchild() is None:                   
                    return v
                else:
                    v = v.get_leftchild()
            if v.get_key() < key:                  
                if v.get_rightchild() is None:                  
                    return v
                else:
                    v = v.get_rightchild()
                    
    def search(self, key):
        v = self.root
        while v is not None:
            if key == v.get_key(): 
                return v
            elif key < v.get_key():
                v = v.get_leftchild()
            elif key > v.get_key():
                v = v.get_rightchild()
        return None
    
    def searchParent(self, key):
        parent = None
        if self.search(key) is not None:
            parent = self.search(key).get_parent()
        return parent
    
    def searchMax(self):
        v = self.root
        while v.get_rightchild() is not None:
            v = v.get_rightchild()
        return v
    
    def searchPredecessor(self, key):
        
        node = self.search(key)
        if node is None:
            return None
        # 2 subroutine (uso solo quella con il figlio sx)
        # 1) Node ha il figlio sx
        
        if node.get_leftchild() is not None:
            pred = node.get_leftchild()
            while pred.get_rightchild() is not None:
                pred = pred.get_rightchild()
            return pred
        return None
        
        # 2) Node ha figlio destro
        
        
    def delete(self, key):
        node = self.search(key)
        if node is None:
            return None
        
        # Eliminazione di una foglia
        if node.isLeaf(): 
            parent = node.get_parent()
            if parent is not None:
                if parent.get_leftchild() == node:
                    parent.set_leftchild(None)
                    del node
                elif parent.get_rightchild() == node:
                    parent.set_rightchild(None)
                    del node
            # Node is root
            else:
                self.root = None
                del node
                
        # Eliminazione nodo con due figli (DA TERMINARE)
        elif node.get_rightchild() is not None and node.get_leftchild() is not None:
            print("Eliminazione nodo con due figli")
            pred = self.searchPredecessor(node.get_key())
            print(pred)
            ...
            
            
        # Eliminazione nodo con un figlio
        else:
            child_node = node.get_rightchild()
            if node.get_leftchild() is not None and node.get_rightchild() is None:
                child_node = node.get_leftchild()
                
            parent_node = self.searchParent(node.get_key())
            # Node è root
            if parent_node is None:
                self.root = child_node
                self.root.set_element("Root")
                self.root.set_parent(None)
            else:
                print(f"parent of {node.get_key()}: ", parent_node)
                print("Parent left child: ",parent_node.get_leftchild())
                print("Parent right child: ", parent_node.get_rightchild())
                if parent_node.get_leftchild() == node:
                    print("left\n")
                    child_node.set_parent(parent_node)
                    parent_node.set_leftchild(child_node)
                    del node
                elif parent_node.get_rightchild() == node:
                    print("right\n")
                    child_node.set_parent(parent_node)
                    parent_node.set_rightchild(child_node)
                    del node
                
    
    def DFSRec(self, node, level = 0):
        if node is None: return
        print(node, level)
        self.DFSRec(node.get_leftchild(), level+1)
        self.DFSRec(node.get_rightchild(), level+1)
    
    # Ricerca + costruzione albero come dizionario T
    def __DFSRec_T(self, node, level = 0):
        if node is None: return
        # print(node, level)
        if node.get_key() not in self.T:
            self.T[node.get_key()] = {}
        if node.get_leftchild() is not None:
            self.T[node.get_key()][node.get_leftchild().get_key()] = 1
        if node.get_rightchild() is not None:
            self.T[node.get_key()][node.get_rightchild().get_key()] = 1
        
        self.__DFSRec_T(node.get_leftchild(), level+1)
        self.__DFSRec_T(node.get_rightchild(), level+1)
    
    def build_tree_dict(self):
        if self.root is None:
            return None
        self.T = {self.root.get_key(): {}}
        self.__DFSRec_T(self.root)
        return self.T
    
    def __str__(self):
        return "({}),({})".format(self.root.get_key(),self.root.get_element())

    
    
if __name__ == "__main__":
    # data = [1, 3, 7, 9, 2, 4]
    data = [1, 3, 7, 9, 2, 4, 43, 31, 0, 5, 6]
    # data = [6]
    #data = []
    # data = random.sample(range(100), 20) # lista senza valori ripetuti
    data = [58, 4, 87, 18, 43, 99, 13, 2, 16, 29, 27, 53, 15, 84, 82, 5, 71, 81, 35, 34]
    #data = [71]
    print(data)    
    bst = BST(data)
    
    bst.DFSRec(bst.root)
    print("\n")
    
    T_dict = bst.build_tree_dict()
    T1 = networkx.DiGraph(T_dict)    
    pos = graphviz_layout(T1, prog="dot")
    networkx.draw(T1, pos, with_labels=True)
    plt.show()
    
    # bst.delete(2)
    bst.delete(34)
    """bst.delete(71)
    """
    
    
    """
    bst.DFSRec(bst.root)
    print("Parent di 2: ", bst.search(2).get_parent())"""
    """print("Max: ", bst.searchMax())
    bst.delete(6)
    bst.DFSRec(bst.root)"""
    # print(bst)
    # for key in data:
    #     node = bst.search(key)
    #     print("{} {} {}".format(node,
    #                             node.get_leftchild() ,
    #                             node.get_rightchild()))

    
    
    
    
    
    
    