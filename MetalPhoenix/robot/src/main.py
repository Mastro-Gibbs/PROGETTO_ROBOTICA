from agent import Agent
""" Main program """

__agent: Agent = Agent()

try:
    __agent.run()
    __agent.controller.write_data_analysis()

except KeyboardInterrupt:
    print()
    print("TIME: ", Agent.controller.execution_time)
    print("Number of nodes: ", Agent.controller.number_of_nodes)
    print("Number of dead end: ", Agent.controller.number_of_dead_end, "\n")
    print("Priority list: ", Agent.controller.priority_list, "\n")
    print("Tree: ", Agent.controller.tree.build_tree_dict(), "\n")
    print("Trajectory: ", Agent.controller.trajectory, "\n")
    print("Performed commands: ", Agent.controller.performed_commands, "\n")
    print("Performed commands and actions: ", Agent.controller.performed_com_actions, "\n")
    print("---- Exporting data ... ----")
    Agent.controller.write_data_analysis()
    print()
    print("---- DONE! ----", "\n")
    __agent.stop()
