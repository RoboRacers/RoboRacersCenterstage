## Autonomous Module

# Creating Trajectory Sequence Groups in /Trajectories.java

The `Trajectory` class is a static class (meaning it does not have to be instantiated to be accessed).
This allows you to call in any of your autonomous opmodes without having to create a object from the class.
The 

Declare the class as `public static class <insert_name> implements TrajectorySequenceGroup`

Initialize your trajectories and/or trajectorySequences with the `static` modifier as they are used by other `static` functions.

Create a `public static void init()` function. This is required by the `TrajectorySequenceGroup` interface. In this function, build you functions. This means that the functions can be build before they are run.

Then, create a `public static void run()` function. This is also required by the `TrajectorySequenceGroup` class. Here, you can make your trajectories run.

Optionally, you can create a `public static void runAsync()` function. This is not required but you can set the same trajectories you created to run asynchronously.

For an example, check out the first TrajectorySequenceGroup in /Trajectories.java and check out the implementation in /TemplateAutoop.java.

