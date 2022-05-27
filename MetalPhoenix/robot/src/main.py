import agent
""" Main program """

try:
    agent.run()

except KeyboardInterrupt:
    print()
    print("Tree: ", agent.c.tree.build_tree_dict(), "\n")
    print("Performed commands: ", agent.c.performed_commands, "\n")
    print("Trajectory: ", agent.c.trajectory, "\n")
    print("TIME: ", agent.c.time_to_solve)
    print("Number of nodes", agent.c.number_of_nodes)
    print("Number of dead end", agent.c.number_of_dead_end)
    print("Priority list", agent.c.priority_list_data)
    agent.c.write_data_analysis()
    print()

    """
    print("Left values: ", agent.c.left_values, "\n")
    print("Front values: ", c.front_values, "\n")
    print("Right values: ", c.right_values, "\n")
    """
    print()
    agent.stop()
