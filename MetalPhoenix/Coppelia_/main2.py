from tree import Node, Tree, WAY
import random

b = Tree()

b.append(Node(str(random.randint(0, 10000))), WAY.MID)

b.regress()

b.append(Node(str(random.randint(0, 10000))), WAY.LEFT)

b.regress()

b.append(Node(str(random.randint(0, 10000))), WAY.RIGHT)

b.append(Node(str(random.randint(0, 10000))), WAY.MID)

b.append(Node(str(random.randint(0, 10000))), WAY.RIGHT)

b.append(Node(str(random.randint(0, 10000))), WAY.MID)

b.regress()

b.append(Node(str(random.randint(0, 10000))), WAY.MID)

b.append(Node(str(random.randint(0, 10000))), WAY.RIGHT)

b.append(Node(str(random.randint(0, 10000))), WAY.MID)


print(b.build_tree_dict())





