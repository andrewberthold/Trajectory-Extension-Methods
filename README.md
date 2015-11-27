# Trajectory-Extension-Methods
A collection of extension methods for predicting and drawing the trajectories of Rigidbody and Rigidbody2D objects in Unity

USAGE:

Since they are extension methods they can be called directly from Rigidbody or Rigidbody2D instances eg:

    Rigidbody MyRigidbodyInstance = GetComponent<Rigidbody>();
    MyRigidbodyInstance.DebugTrajectory(Force, ForceMode, Color.Green, 3.0f, 3.0f);
    MyRigidbodyInstance.AddForce(Force, ForceMode);
  
Alternatively, you can call the methods from the Trajectory class itself.

    Trajectory.DebugTrajectory(MyRigidbodyInstance, Force, ForceMode, Color.green, 3.0f, 3.0f);


METHODS:

GetTrajectory - returns an array of position vectors for each point along the trajectory. There is also an Out parameter for getting an array of the expected velocities that the rigidbody should have at each point along the trajectory.

DrawTrajectory - Draws a trajectory using Gizmos.DrawLine. More efficient than DebugTrajectory, but has less options.

DebugTrajectory - Draws a trajectory using Debug.DrawLine. Allows for drawing lines that persist for more than 1 frame and has depthTest parameter.


PARAMETERS:

rb - Rigidbody or Rigidbody2D : The body whose trajectory is being drawn.

force - Vector3 or Vector2 : Vector of the force being predicted. If not trying to predict a force, use Vector3.zero or Vector2.zero.

mode - ForceMode or ForceMode2D : Force Mode of the force being predicted. Irrelevant if using a zero vector force.

color - Color : Color of the line being drawn.

out velocities - Vector3[] or Vector2[] : Outputs an array of vectors containing the expected velocities at each position in the trajectory.

trajectoryDuration - float : Amount of time in seconds to predict trajectory.

lineDuration - float : Amount of time in seconds the drawn line will persist.

constantForce - bool : For predicting forces that are applied every FixedUpdate? Eg, true for predicting the trajectory of a rocket, false for predicting the trajectory of a cannon ball.

depthTest - bool : Whether or not the line should be faded when behind other objects.


NOTE:

Since I didn't know precisely how and in what order unity's 3D "PhysX" and 2D "Box2D" engines apply their forces, It was mostly guess work. Both 2D and 3D predictions begin to fall apart when the drag is higher than the Physics Framerate. Apart from that i haven't been able to find any errors with the 3D trajectory, even when using absurd values for mass, timestep, force etc... The 2D predictions however reveal slight inaccuracies when traveling at very fast speeds over huge distances, the margin of error is probably only ~0.1 - 0.5% and shouldn't be noticeable in normal use cases.
