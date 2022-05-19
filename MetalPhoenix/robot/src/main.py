import agent

try:
    agent.run()

except KeyboardInterrupt:
    print("\n")
    print("Tree: ", agent.c.tree.build_tree_dict(), "\n")
    print("Performed commands: ", agent.c.performed_commands, "\n")
    print("Trajectory: ", agent.c.trajectory, "\n")
    """
    print("Left values: ", agent.c.left_values, "\n")
    print("Front values: ", c.front_values, "\n")
    print("Right values: ", c.right_values, "\n")
    """
    print()
    agent.stop()
