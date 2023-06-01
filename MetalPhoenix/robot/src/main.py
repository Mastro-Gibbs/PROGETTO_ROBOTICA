from agent import agent
""" Main program """

__agent: agent = agent()

try:
    __agent.run()
    __agent.controller.write_data_analysis()

except KeyboardInterrupt:
    print()
    print("TIME: ", agent.controller.execution_time)

    print("Number of nodes: ", agent.controller.number_of_nodes)
    print("Number of dead end: ", agent.controller.number_of_dead_end)
    print("Priority list: ", agent.controller.priority_list, "\n")
    print("Tree: ", agent.controller.tree.build_tree_dict(), "\n")
    print("Trajectory: ", agent.controller.trajectory, "\n")
    print("Performed commands: ", agent.controller.performed_commands, "\n")
    print("Performed commands and actions: ", agent.controller.performed_com_actions, "\n")
    print("---- Exporting data ... ----")
    agent.controller.write_data_analysis()
    print()
    print("---- DONE! ----")
    print()

    """
    print("Left values: ", agent.controller.left_values, "\n")
    print("Front values: ", controller.front_values, "\n")
    print("Right values: ", controller.right_values, "\n")
    """
    print()
    __agent.stop()
