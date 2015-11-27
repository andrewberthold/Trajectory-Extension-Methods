using UnityEngine;

/// <summary>
///     A collection of extension methods for <seealso cref="Rigidbody" /> and <seealso cref="Rigidbody2D" /> that
///     calculate and get/draw expected trajectories.
/// </summary>
/// <remarks>
///     The methods take either a <seealso cref="Vector3" /> force and a <seealso cref="ForceMode" /> for
///     <seealso cref="Rigidbody" />, or a <seealso cref="Vector2" /> force and a <seealso cref="ForceMode2D" /> for
///     <seealso cref="Rigidbody2D" />. They can also be passed a Vector2.zero or Vector3.zero, for calculating the
///     trajectory using only the objects current velocity and regular physics forces without any additonal forces.
///     When using these zero vectors the forcemode is irrelevant.<para />
/// 
///     For RigidBody, the calculations act upon <seealso cref="Physics" />'s Gravity,
///     and the Rigidbodys's Mass, Linear Drag, UseGravity and Freeze Position Constraints. 
///     For Rigidbody2D, the calculations act upon <seealso cref="Physics2D" />'s Gravity 
///     and MaxTranslationSpeed, and the Rigidbody2D's Mass, Linear Drag, Gravity Scale and 
///     Freeze Position Constraints.<para />
/// 
///     2D trajectory prediction may become innaccurate when using extremely high linear drag values or if the 
///     velocity exceeds Physics2D.MaxTranslationSpeed.  
/// </remarks>
public static class Trajectory
{
    #region GetTrajectory2D Methods

    /// <summary>
    ///     Calculates a <seealso cref="Rigidbody2D" />'s trajectory and returns it as an array of position vectors.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody2D.AddForce() method. Use Vector2.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector2.zero. </param>
    /// <param name="velocities"> An array of the body's expected velocities at each point along the trajectory. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <returns>
    ///     An array of the body's expected positions after each FixedUpdate in the trajectory. The last index being the
    ///     final position.
    /// </returns>
    public static Vector2[] GetTrajectory(this Rigidbody2D rb, Vector2 force, ForceMode2D mode, out Vector2[] velocities,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        var maxSpeedErrorThrown = false;
        var physicsFramerate = 1 / Time.fixedDeltaTime;

        if (rb.drag >= physicsFramerate)
            Debug.LogWarning(rb + " Linear Drag(" + rb.drag + ") too high, trajectory Prediction will be inaccurate."
                             + " Maximum safe drag = (1 / Time.fixedDeltaTime), currently: " + (physicsFramerate) +
                             ").");

        trajectoryDuration = Mathf.Max(Time.fixedDeltaTime * 2, trajectoryDuration);
        var points = Mathf.Max(2, Mathf.RoundToInt(trajectoryDuration / Time.fixedDeltaTime));

        var positions = new Vector2[points];
        velocities = new Vector2[points];

        positions[0] = rb.transform.position;
        velocities[0] = rb.velocity;
        
        // As far as i can tell... (please correct me if i'm wrong)
        // Rigidbody2D.AddForce() applies forces after gravity and drag have already been applied.
        // Rigidbody.AddForce() applies forces before gravity and drag.
        // This only has a noticable difference when comparing large forces against objects with high drag as
        // Rigidbody2D.AddForce is not immediately effected by drag, Rigidbody.AddForce is.

        //Rigidbody2D order:
        //Gravity
        //Drag
        //AddForce()
        //Constraints

        for (var i = 0; i < positions.Length - 1; i++)
        {
            var updatedVelocity = velocities[i];

            updatedVelocity = updatedVelocity.ApplyGravity(rb);
            updatedVelocity = updatedVelocity.ApplyDrag(rb);

            if (i == 0 || constantForce)
                updatedVelocity = updatedVelocity.ApplyForce(rb, force, mode);

            if (!maxSpeedErrorThrown && updatedVelocity.magnitude > Physics2D.maxTranslationSpeed * physicsFramerate)
            {
                Debug.Log(rb + "is expected to reach a speed (" + updatedVelocity.magnitude * Time.fixedDeltaTime +
                          ") greater than Physics2D.maxTranslationSpeed(" + Physics2D.maxTranslationSpeed +
                          ") trajectory prediction may be innaccurate, Consider increasing maxTranslationSpeed," +
                          " mass, linear drag or decreasing manually applied forces");
                maxSpeedErrorThrown = true;
            }

            updatedVelocity = updatedVelocity.ApplyConstraints(rb);

            velocities[i] = updatedVelocity;

            positions[i + 1] = positions[i] + (velocities[i] * Time.fixedDeltaTime);
            velocities[i + 1] = velocities[i];
        }

        return positions;
    }

    /// <summary>
    ///     Calculates a <seealso cref="Rigidbody2D" />'s trajectory and returns it as an array of position vectors.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody2D.AddForce() method. Use Vector2.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector2.zero. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <returns>
    ///     An array of the body's expected positions after each FixedUpdate in the trajectory. The last index being the
    ///     final position.
    /// </returns>
    public static Vector2[] GetTrajectory(this Rigidbody2D rb, Vector2 force, ForceMode2D mode,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        Vector2[] velocities;
        return rb.GetTrajectory(force, mode, out velocities, trajectoryDuration, constantForce);
    }

    #endregion

    #region GetTrajectory3D Methods

    /// <summary>
    ///     Calculates a <seealso cref="Rigidbody" />'s trajectory and returns it as an array of position vectors.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody.AddForce() method. Use Vector3.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector3.zero. </param>
    /// <param name="velocities"> An array of the body's expected velocities at each point along the trajectory. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <returns>
    ///     An array of the body's expected positions after each FixedUpdate in the trajectory. The last index being the
    ///     final position.
    /// </returns>
    public static Vector3[] GetTrajectory(this Rigidbody rb, Vector3 force, ForceMode mode, out Vector3[] velocities,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        var physicsFramerate = 1 / Time.fixedDeltaTime;
        if (rb.drag >= physicsFramerate)
            Debug.LogWarning(rb + " Linear Drag(" + rb.drag + ") too high, trajectory Prediction may be inaccurate."
                             + " Maximum safe drag = (1 / Time.fixedDeltaTime), currently: " + (physicsFramerate) +
                             ").");

        trajectoryDuration = Mathf.Max(Time.fixedDeltaTime * 2, trajectoryDuration);

        var points = Mathf.Max(2, Mathf.RoundToInt(trajectoryDuration / Time.fixedDeltaTime));

        var positions = new Vector3[points];
        velocities = new Vector3[points];

        positions[0] = rb.transform.position;
        velocities[0] = rb.velocity;

        // As far as i can tell... (please correct me if i'm wrong)
        // Rigidbody2D.AddForce() applies forces after gravity and drag have already been applied.
        // Rigidbody.AddForce() applies forces before gravity and drag.
        // This only has a noticable difference when comparing large forces against objects with high drag as
        // Rigidbody2D.AddForce is not immediately effected by drag, Rigidbody.AddForce is.

        //Rigidbody order:
        //AddForce()
        //Gravity
        //Drag
        //Constraints

        for (var i = 0; i < positions.Length - 1; i++)
        {
            var updatedVelocity = velocities[i];


            if (i == 0 || constantForce)
                updatedVelocity = updatedVelocity.ApplyForce(rb, force, mode);

            updatedVelocity = updatedVelocity.ApplyGravity(rb);
            updatedVelocity = updatedVelocity.ApplyDrag(rb);
            updatedVelocity = updatedVelocity.ApplyConstraints(rb);

            velocities[i] = updatedVelocity;

            positions[i + 1] = positions[i] + (velocities[i] * Time.fixedDeltaTime);
            velocities[i + 1] = velocities[i];
        }
        return positions;
    }

    /// <summary>
    ///     Calculates a <seealso cref="Rigidbody" />'s trajectory and returns it as an array of position vectors.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody.AddForce() method. Use Vector3.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector3.zero. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <returns>
    ///     An array of the body's expected positions after each FixedUpdate in the trajectory. The last index being the
    ///     final position.
    /// </returns>
    public static Vector3[] GetTrajectory(this Rigidbody rb, Vector3 force, ForceMode mode,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        Vector3[] velocities;
        return rb.GetTrajectory(force, mode, out velocities, trajectoryDuration, constantForce);
    }

    #endregion

    #region DebugTrajectory Methods

    /// <summary>
    ///     Draws a <seealso cref="Rigidbody2D" />'s trajectory using <seealso cref="Debug" />.DrawLine.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody2D.AddForce() method. Use Vector2.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector2.zero. </param>
    /// <param name="color"> The color of the line being drawn. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="lineDuration"> Amount of time in seconds the drawn line will persist. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <param name='depthTest'> Whether or not the line should be faded when behind other objects. </param>
    public static void DebugTrajectory(this Rigidbody2D rb, Vector2 force, ForceMode2D mode, Color color,
        float trajectoryDuration = 1.0f, float lineDuration = 0.0f, bool constantForce = false, bool depthTest = false)
    {
        var positions = rb.GetTrajectory(force, mode, trajectoryDuration, constantForce);
        for (var i = 0; i < positions.Length - 1; i++)
            Debug.DrawLine(positions[i], positions[i + 1], color, lineDuration, depthTest);
    }
    
    /// <summary>
    ///     Draws a <seealso cref="Rigidbody" />'s trajectory using <seealso cref="Debug" />.DrawLine.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody.AddForce() method. Use Vector3.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector3.zero. </param>
    /// <param name="color"> The color of the line being drawn. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="lineDuration"> Amount of time in seconds the drawn line will persist. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    /// <param name='depthTest'> Whether or not the line should be faded when behind other objects. </param>
    public static void DebugTrajectory(this Rigidbody rb, Vector3 force, ForceMode mode, Color color,
        float trajectoryDuration = 1.0f, float lineDuration = 0.0f, bool constantForce = false, bool depthTest = false)
    {
        var positions = rb.GetTrajectory(force, mode, trajectoryDuration, constantForce);
        for (var i = 0; i < positions.Length - 1; i++)
            Debug.DrawLine(positions[i], positions[i + 1], color, lineDuration, depthTest);
    }

    #endregion

    #region DrawTrajectory Methods

    /// <summary>
    ///     Draws a <seealso cref="Rigidbody2D" />'s trajectory using <seealso cref="Gizmos" />.DrawLine.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody2D.AddForce() method. Use Vector2.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector2.zero. </param>
    /// <param name="color"> The color of the line being drawn. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    public static void DrawTrajectory(this Rigidbody2D rb, Vector2 force, ForceMode2D mode, Color color,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        var oldColor = Gizmos.color;
        Gizmos.color = color;

        var positions = rb.GetTrajectory(force, mode, trajectoryDuration, constantForce);
        for (var i = 0; i < positions.Length - 1; i++)
            Gizmos.DrawLine(positions[i], positions[i + 1]);

        Gizmos.color = oldColor;
    }

    /// <summary>
    ///     Draws a <seealso cref="Rigidbody" />'s trajectory using <seealso cref="Gizmos" />.DrawLine.
    /// </summary>
    /// <param name="rb"> The body whose trajectory is being drawn. </param>
    /// <param name="force"> For predicting the effects of a Rigidbody.AddForce() method. Use Vector3.zero if not needed. </param>
    /// <param name="mode"> Determines how the force vector changes the velocity. Irrelevant when using Vector3.zero. </param>
    /// <param name="color"> The color of the line being drawn. </param>
    /// <param name="trajectoryDuration"> Amount of time in seconds to predict. </param>
    /// <param name="constantForce"> Will the force be applied every FixedUpdate. </param>
    public static void DrawTrajectory(this Rigidbody rb, Vector3 force, ForceMode mode, Color color,
        float trajectoryDuration = 1.0f, bool constantForce = false)
    {
        var oldColor = Gizmos.color;
        Gizmos.color = color;

        var positions = rb.GetTrajectory(force, mode, trajectoryDuration, constantForce);
        for (var i = 0; i < positions.Length - 1; i++)
            Gizmos.DrawLine(positions[i], positions[i + 1]);

        Gizmos.color = oldColor;
    }

    #endregion

    #region Helper Methods for GetTrajectory

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the gravity forces
    ///     which the given Rigidbody2D would recieve, then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify </param>
    /// <param name="rb"> The Rigidbody2D used to calculate the gravity forces. </param>
    /// <returns> Velocity vector with gravity applied. </returns>
    private static Vector2 ApplyGravity(this Vector2 velocity, Rigidbody2D rb)
    {
        var gravity = Physics2D.gravity;
        var gravityScale = rb.gravityScale;

        if (rb.constraints == RigidbodyConstraints2D.FreezePositionX)
            gravity.x = 0;
        if (rb.constraints == RigidbodyConstraints2D.FreezePositionY)
            gravity.y = 0;

        var newVelocity = velocity + (gravity * gravityScale * Time.fixedDeltaTime);
        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the gravity forces
    ///     which the given Rigidbody would recieve, then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify </param>
    /// <param name="rb"> The Rigidbody used to calculate the gravity forces. </param>
    /// <returns> Velocity vector with gravity applied. </returns>
    private static Vector3 ApplyGravity(this Vector3 velocity, Rigidbody rb)
    {
        var gravity = Physics.gravity;

        if (rb.useGravity == false)
            gravity = Vector3.zero;

        if (rb.constraints == RigidbodyConstraints.FreezePositionX)
            gravity.x = 0;
        if (rb.constraints == RigidbodyConstraints.FreezePositionY)
            gravity.y = 0;
        if (rb.constraints == RigidbodyConstraints.FreezePositionZ)
            gravity.z = 0;

        var newVelocity = velocity + (gravity * Time.fixedDeltaTime);
        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the effect of
    ///     the Rigidbody's drag, then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify </param>
    /// <param name="rb"> The Rigidbody used to calculate the drag forces. </param>
    /// <returns> Velocity vector with drag applied. </returns>
    private static Vector2 ApplyDrag(this Vector2 velocity, Rigidbody2D rb)
    {
        var newVelocity = velocity;
        var drag = (1 - (rb.drag * Time.fixedDeltaTime));

        newVelocity *= drag;
        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the effect of
    ///     the Rigidbody's drag, then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify </param>
    /// <param name="rb"> The Rigidbody used to calculate the drag forces. </param>
    /// <returns> Velocity vector with drag applied. </returns>
    private static Vector3 ApplyDrag(this Vector3 velocity, Rigidbody rb)
    {
        var newVelocity = velocity;
        var drag = (1 - (rb.drag * Time.fixedDeltaTime));

        newVelocity *= drag;
        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the velocity change amount
    ///     that would be caused by a force with given forcemode against given Rigidbody2D.
    ///     Then Returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify. </param>
    /// <param name="rb"> The Rigidbody2D used to calculate how the force is applied. </param>
    /// <param name="force"> The force vector to apply. </param>
    /// <param name="mode"> Determines how the force translates into a change in velocity. </param>
    /// <returns> Velocity vector with force applied. </returns>
    private static Vector2 ApplyForce(this Vector2 velocity, Rigidbody2D rb, Vector2 force, ForceMode2D mode)
    {
        var newVelocity = velocity;

        switch (mode)
        {
            case ForceMode2D.Force:
                newVelocity += (force * Time.fixedDeltaTime) / rb.mass;
                break;
            case ForceMode2D.Impulse:
                newVelocity += (force) / rb.mass;
                break;
        }

        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector, calculates and adds the velocity change amount
    ///     that would be caused by a force with given forcemode against given Rigidbody.
    ///     Then Returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify. </param>
    /// <param name="rb"> The Rigidbody used to calculate how the force is applied. </param>
    /// <param name="force"> The force vector to apply. </param>
    /// <param name="mode"> Determines how the force translates into a change in velocity. </param>
    /// <returns> Velocity vector with force applied. </returns>
    private static Vector3 ApplyForce(this Vector3 velocity, Rigidbody rb, Vector3 force, ForceMode mode)
    {
        var newVelocity = velocity;

        switch (mode)
        {
            case ForceMode.Force:
                newVelocity += (force * Time.fixedDeltaTime) / rb.mass;
                break;
            case ForceMode.Acceleration:
                newVelocity += force * Time.fixedDeltaTime;
                break;
            case ForceMode.Impulse:
                newVelocity += force / rb.mass;
                break;
            case ForceMode.VelocityChange:
                newVelocity += force;
                break;
        }

        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector and limits it according to a
    ///     RigidBody2D's constraints then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify. </param>
    /// <param name="rb"> The Rigidbody2D whose constraints will be used against the vector </param>
    /// <returns> Velocity vector with constraints applied. </returns>
    private static Vector2 ApplyConstraints(this Vector2 velocity, Rigidbody2D rb)
    {
        var newVelocity = velocity;

        if ((rb.constraints & RigidbodyConstraints2D.FreezePositionX) == RigidbodyConstraints2D.FreezePositionX)
            newVelocity.x = 0;
        if ((rb.constraints & RigidbodyConstraints2D.FreezePositionY) == RigidbodyConstraints2D.FreezePositionY)
            newVelocity.y = 0;

        newVelocity = Vector2.ClampMagnitude(newVelocity, Physics2D.maxTranslationSpeed * (1 / Time.fixedDeltaTime));

        return newVelocity;
    }

    /// <summary>
    ///     Takes a velocity vector and limits it according to a
    ///     RigidBody's constraints then returns the result.
    /// </summary>
    /// <param name="velocity"> The velocity vector to modify. </param>
    /// <param name="rb"> The Rigidbody whose constraints will be used against the vector </param>
    /// <returns> Velocity vector with constraints applied. </returns>
    private static Vector3 ApplyConstraints(this Vector3 velocity, Rigidbody rb)
    {
        var newVelocity = velocity;

        if ((rb.constraints & RigidbodyConstraints.FreezePositionX) == RigidbodyConstraints.FreezePositionX)
            newVelocity.x = 0;
        if ((rb.constraints & RigidbodyConstraints.FreezePositionY) == RigidbodyConstraints.FreezePositionY)
            newVelocity.y = 0;
        if ((rb.constraints & RigidbodyConstraints.FreezePositionZ) == RigidbodyConstraints.FreezePositionZ)
            newVelocity.z = 0;

        return newVelocity;
    }

    #endregion
}