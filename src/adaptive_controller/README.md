# adaptive_controller
A ROS context aware reasoner based on ROSoClingo.

The system is composed by a Context Middleware node, which receives high level information from the other system components and estimates a context model, and a main Reasoner, which elaborates plans for the system according to its static knowledge and to the context estimate.

To test the system on shell:
roslaunch --screen adaptive_controller adaptive_controller.launch
