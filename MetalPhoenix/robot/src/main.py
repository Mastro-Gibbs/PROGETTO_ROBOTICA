
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

    agent.stop()
    print("EXPIRED")


"""
from physical_body import PhysicalBody, ThreadType

if __name__ == "__main__":
    pb = PhysicalBody()
    try:
        pb.thread_begin(ThreadType.th_prox)
        pb.thread_begin(ThreadType.th_orientation)

        while True:
            print("FRONT: {0}, LEFT: {1}, RIGHT: {2}, BACK: {3}, GATE: {4}".format(pb.get_proxF(), pb.get_proxL(), pb.get_proxR(), pb.get_proxB(), pb.get_gate()))

    except KeyboardInterrupt:
        pb.safe_exit()
"""