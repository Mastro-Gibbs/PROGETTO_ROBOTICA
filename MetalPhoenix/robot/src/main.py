import agent

try:
    agent.run()

except KeyboardInterrupt:
    """ print("\n")
    print("Tree: ", c.tree.build_tree_dict(), "\n")
    print("Performed commands: ", c.performed_commands, "\n")
    print("Trajectory: ", c.trajectory, "\n")
    print("Left values: ", c.left_values, "\n")
    print("Front values: ", c.front_values, "\n")
    print("Right values: ", c.right_values, "\n")"""

    print()
    agent.stop()
