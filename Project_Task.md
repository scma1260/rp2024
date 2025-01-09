## Project Assignment: Pushing While Avoiding Obstacles

### Objective
Your task is to implement and solve a robotic manipulation task using the PyBullet simulation framework from the lectures. You will train a model using simulated data to solve the task. You can choose between two approaches for training your model:

- **Imitation Learning**: Train your model by learning from pre-recorded demonstrations.  
- **Reinforcement Learning**: Design a reward function to train the model through trial and error.

If you choose to use imitation learning, you will need to generate data in advance. You can either use an oracle similar to the one presented in the lecture to generate scripted demonstrations or implement your own method using human input for demonstrations.

If you choose to use reinforcement learning, you will need to design an appropriate reward function to guide the learning process.

You can adapt the simulation framework as required.

### Task
**Pushing While Avoiding Obstacles**: The workspace contains multiple objects that need to be pushed into a target area, but there are obstacles placed randomly throughout the environment. The task is for the robot to push the objects to the target area while avoiding collisions with the obstacles. The objects can be of different shapes, colors, and sizes, while the obstacles should be of a uniform color.

### Design Considerations
- Think about the actions the robot must execute to solve the task.  
- Choose a suitable model for the task, considering factors like state space, action space, and your chosen method (imitation learning or reinforcement learning).  
- Document your design decisions and the rationale behind your choices.

### Deliverables

**1. Code**: Submit your code in a well-organized GitHub repository. Include a README file with instructions on setting up and running your project.

**2. Presentation**: Present your results in one of the following formats:  
   - A PowerPoint presentation  
   - A Jupyter Notebook

**3. Documentation**: In your presentation or notebook, include the following:
   - The rationale behind your choice of algorithm (imitation learning or reinforcement learning).
   - The steps and considerations taken to improve policy convergence.
   - Any challenges faced during the project and how you addressed them.
   - Analysis of the results, including any metrics used to evaluate the performance of your model.
