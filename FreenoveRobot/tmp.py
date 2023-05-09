self.__logger.log(f"--MODE: {self.__machine.mode}")
            self.__logger.log(f"--ACTIONS: {com_actions}")
            self.__logger.log(f"--ACTION: {com_action}")
            self.__logger.log(f"--CURRENT NODE: {self.__maze.tree.current}")


self.__logger.log("--CURRENT TREE:", Color.GRAY)
            self.__logger.log(f"{self.__maze.tree.build_tree_dict()}", Color.GRAY, noheader=True)
            self.__logger.log(f"--CURRENT NODE: {self.__maze.tree.current}", Color.GRAY, newline=True)
            self.__logger.log(f"--Available actions: {com_actions}", Color.GREEN)

self.__logger.log(f"--(STATE, POSITION): ({self.__machine.state}, {self.__machine.position})",
                              Color.GRAY)
            self.__logger.log(f"--Performing action: {com_action}", Color.GRAY)


self.__logger.log(f"--(STATE, POSITION): ({self.__machine.state}, {self.__machine.position})",
                              Color.GRAY)