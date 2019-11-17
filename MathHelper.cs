using UnityEngine;

/// <summary>
/// Contains helper functions for math.
/// </summary>
public static class MathHelper {

    #region General

    /// <summary>
    /// Returns a sin in radians (but before it is multiplied by pi) using two vectors. -1 to 1
    /// </summary>
    /// <returns>The return value is in radians normalized (-1 to 1)..</returns>
    public static float SinNormalized(Vector2 a, Vector2 b)
    {
        return Mathf.Sin(Vector2.Angle(a, b) * Mathf.Deg2Rad);
    }

    /// <summary>
    /// Returns a cos in radians (but before it is multiplied by pi) using two vectors. -1 to 1.
    /// </summary>
    public static float CosNormalized(Vector2 a, Vector2 b)
    {
        return Mathf.Cos(Vector2.Angle(a, b) * Mathf.Deg2Rad);
    }

    /// <summary>
    /// Determines the 3rd side of a triangle, given the other 2 sides and the angle opposite the side being determined.
    /// </summary>
    public static float CalcTriangleSideCos (float angleAtOppositeCorner, float sideA, float sideB)
    {
        return Mathf.Sqrt(sideA * sideA + sideB * sideB - 2 * sideA * sideB * Mathf.Cos(angleAtOppositeCorner));
    }

    /// <summary>
    /// Determines the 3rd side of a triangle, given one other sides, the angle opposite that side, and the angle opposite the side being determined.
    /// </summary>
    public static float CalcTriangleSideSin (float angleAtOppositeCorner, float sideA, float angleAtOppositeCornerofA)
    {
        return Mathf.Sin(angleAtOppositeCorner) * sideA / Mathf.Sin(angleAtOppositeCornerofA);
    }

    /// <summary>
    /// Determines an angle on a triangle, given all the sides of a triangle.
    /// <returns>Angle in radians.</returns>
    public static float CalcTriangleAngleCos (float oppositeSide, float sideA, float sideB)
    {
        return Mathf.Acos((sideA * sideA + sideB * sideB - oppositeSide * oppositeSide) / 2 * sideA * sideB);
    }

    /// <summary>
    /// Determines an angle on a triangle, given 2 side lengths and an angle opposite one of those sides.
    /// </summary>
    /// <returns>Angle in radians.</returns>
    public static float CalcTriangleAngleSin(float oppositeSide, float sideA, float angleAtOppositeCornerofA)
    {
        return Mathf.Asin(oppositeSide * Mathf.Sin(angleAtOppositeCornerofA) / sideA);
    }

    /// <summary>
    /// Returns the interpolation between 2 points.
    /// An interpolation values between 0-1 will return a point between the two given points.
    /// </summary>
    public static Vector2 Interpolate (Vector2 a, Vector2 b, float interpolation)
    {
        return b * interpolation + a * (1 - interpolation);
    }

    /// <summary>
    /// Returns the interpolation between 2 points.
    /// An interpolation values between 0-1 will return a point between the two given points.
    /// </summary>
    public static Vector3 Interpolate(Vector3 a, Vector3 b, float interpolation)
    {
        return b * interpolation + a * (1 - interpolation);
    }

    /// <summary>
    /// Returns the closest point on a line to the given point.
    /// </summary>
    public static Vector2 ClampToLine(Vector2 start, Vector2 end, Vector2 point)
    {
        Vector2 line = (end - start).normalized;

        float dot = Vector2.Dot(line, point - start);
        if (dot <= 0)
        {
            return start;
        }
        else if (dot >= Vector2.Distance(start, end))
        {
            return end;
        }
        return start + (line * dot);
    }

    /// <summary>
    /// Returns the closest point on a line to the given point.
    /// </summary>
    public static Vector3 ClampToLine(Vector3 start, Vector3 end, Vector3 point)
    {
        Vector3 line = (end - start).normalized;

        float dot = Vector3.Dot(line, point - start);
        if (dot <= 0)
        {
            return start;
        }
        else if (dot >= Vector3.Distance(start, end))
        {
            return end;
        }
        return start + (line * dot);
    }

    /// <summary>
    /// Calculates the position of an object along a trajectory.
    /// </summary>
    public static Vector2 CalcTrajectoryPoint(Vector2 start, Vector2 velocity, float timePassed)
    {
        return start + velocity * timePassed;
    }

    /// <summary>
    /// Calculates the position of an object along a trajectory.
    /// </summary>
    public static Vector3 CalcTrajectoryPoint(Vector3 start, Vector3 velocity, float timePassed)
    {
        return start + velocity * timePassed;
    }

    /// <summary>
    /// Calculates the position of an object along a trajectory.
    /// </summary>
    public static Vector2 CalcTrajectoryPoint (Vector2 start, Vector2 startVelocity, float timePassed, Vector2 acceleration)
    {
        return start + (2f * startVelocity + acceleration * timePassed) * 0.5f * timePassed;
    }

    /// <summary>
    /// Calculates the position of an object along a trajectory.
    /// </summary>
    public static Vector3 CalcTrajectoryPoint(Vector3 start, Vector3 startVelocity, float timePassed, Vector3 acceleration)
    {
        return start + (2f * startVelocity + acceleration * timePassed) * 0.5f * timePassed;
    }

    /// <summary>
    /// Raycasts along a given trajectory and returns information on any collision, if any.
    /// The number of raycasts that will happen is equal to timePassed / timeStep.
    /// If no collision occurs, an empty RaycastHit2D will be returned.
    /// With layermasks, to hit a specific layer you can use ~LayerMask.NameToLayer("Layer Name")
    /// </summary>
    public static RaycastHit2D RaycastTrajectory(Vector2 start, Vector2 startVelocity, Vector2 acceleration, float timePassed, float timeStep, LayerMask mask)
    {
        if (timePassed <= 0f || timeStep <= 0f)
        {
            return new RaycastHit2D();
        }

        int iterations = Mathf.CeilToInt(timePassed / timeStep);
        RaycastHit2D hit;
        Vector2 pointA = start;
        Vector2 pointB;
        Vector2 distance;

        for (int i = 1; i <= iterations; ++i)
        {
            pointB = CalcTrajectoryPoint(start, startVelocity, i * timeStep, acceleration);
            distance = pointB - pointA;
            hit = Physics2D.Raycast(pointA, distance.normalized, distance.magnitude, mask);

            // Remove this comment to see all these raycasts in the editor
            // Debug.DrawLine(pointA, pointB, Color.red, 1f);

            if (hit.transform != null)
            {
                hit.distance = Vector2.Distance(start, hit.transform.position);
                hit.fraction = (iterations * timeStep) / timePassed;
                return hit;
            }
            pointA = pointB;
        }
        return new RaycastHit2D();
    }

    /// <summary>
    /// Raycasts along a given trajectory and returns information on any collision, if any.
    /// The number of raycasts that will happen is equal to timePassed / timeStep.
    /// If no collision occurs, an empty RaycastHit2D will be returned.
    /// With layermasks, to hit a specific layer you can use ~LayerMask.NameToLayer("Layer Name")
    /// </summary>
    public static RaycastHit RaycastTrajectory(Vector3 start, Vector3 startVelocity, Vector3 acceleration, float timePassed, float timeStep, LayerMask mask)
    {
        if (timePassed <= 0f || timeStep <= 0f)
        {
            return new RaycastHit();
        }

        int iterations = Mathf.CeilToInt(timePassed / timeStep);
        RaycastHit hit;
        Vector3 pointA = start;
        Vector3 pointB;
        Vector3 distance;

        for (int i = 0; i < iterations; ++i)
        {
            pointB = CalcTrajectoryPoint(start, startVelocity, i * timeStep, acceleration);
            distance = pointB - pointA;

            // Remove this comment to see all these raycasts in the editor
            // Debug.DrawLine(pointA, pointB, Color.red, 1f);

            if (Physics.Raycast(pointA, distance.normalized, out hit, distance.magnitude, mask))
            {
                hit.distance = Vector2.Distance(start, hit.transform.position);
                return hit;
            }
            pointA = pointB;
        }
        return new RaycastHit();
    }


    #endregion
}
