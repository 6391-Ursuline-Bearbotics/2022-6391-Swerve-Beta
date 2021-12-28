// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;

public final class TrajectoryGenerator6391 {
  private static final Trajectory kDoNothingTrajectory =
      new Trajectory(Arrays.asList(new Trajectory.State()));
  private static BiConsumer<String, StackTraceElement[]> errorFunc;
  private static ArrayList<Rotation2d> pointHeadings = new ArrayList<Rotation2d>();

  /** Private constructor because this is a utility class. */
  private TrajectoryGenerator6391() {}

  private static void reportError(String error, StackTraceElement[] stackTrace) {
    if (errorFunc != null) {
      errorFunc.accept(error, stackTrace);
    } else {
      MathSharedStore.reportError(error, stackTrace);
    }
  }

  /**
   * Set error reporting function. By default, DriverStation.reportError() is used.
   *
   * @param func Error reporting function, arguments are error and stackTrace.
   */
  public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
    errorFunc = func;
  }

  /**
   * Generates a trajectory from the given control vectors and config. This method uses clamped
   * cubic splines -- a method in which the exterior control vectors and interior waypoints are
   * provided. The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param initial The initial control vector.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending control vector.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      Spline.ControlVector initial,
      List<Translation2d> interiorWaypoints,
      Spline.ControlVector end,
      TrajectoryConfig6391 config) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

    // Clone the control vectors.
    var newInitial = new Spline.ControlVector(initial.x, initial.y);
    var newEnd = new Spline.ControlVector(end.x, end.y);

    // Change the orientation if reversed.
    if (config.isReversed()) {
      newInitial.x[1] *= -1;
      newInitial.y[1] *= -1;
      newEnd.x[1] *= -1;
      newEnd.y[1] *= -1;
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.getCubicSplinesFromControlVectors(
                  newInitial, interiorWaypoints.toArray(new Translation2d[0]), newEnd));
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer6391.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses clamped cubic
   * splines -- a method in which the initial pose, final pose, and interior waypoints are provided.
   * The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig6391 config) {
    var controlVectors =
        SplineHelper.getCubicControlVectorsFromWaypoints(
            start, interiorWaypoints.toArray(new Translation2d[0]), end);

    // Return the generated trajectory.
    return generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config);
  }

  /**
   * Generates a trajectory from the given quintic control vectors and config. This method uses
   * quintic hermite splines -- therefore, all points must be represented by control vectors.
   * Continuous curvature is guaranteed in this method.
   *
   * @param controlVectors List of quintic control vectors.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      ControlVectorList controlVectors, TrajectoryConfig6391 config) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
    final var newControlVectors = new ArrayList<Spline.ControlVector>(controlVectors.size());

    // Create a new control vector list, flipping the orientation if reversed.
    for (final var vector : controlVectors) {
      var newVector = new Spline.ControlVector(vector.x, vector.y);
      if (config.isReversed()) {
        newVector.x[1] *= -1;
        newVector.y[1] *= -1;
      }
      newControlVectors.add(newVector);
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.getQuinticSplinesFromControlVectors(
                  newControlVectors.toArray(new Spline.ControlVector[] {})));
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer6391.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
   * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature is
   * guaranteed in this method.
   *
   * @param waypoints List of waypoints..
   * @param headings List of robot heading at each waypoint.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  @SuppressWarnings("LocalVariableName")
  public static Trajectory generateTrajectory(List<Pose2d> waypoints, ArrayList<Rotation2d> headings, TrajectoryConfig6391 config) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

    List<Pose2d> newWaypoints = new ArrayList<>();
    if (config.isReversed()) {
      for (Pose2d originalWaypoint : waypoints) {
        newWaypoints.add(originalWaypoint.plus(flip));
      }
    } else {
      newWaypoints.addAll(waypoints);
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints), headings);
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer6391.timeParameterizeTrajectory(
        points,
        pointHeadings,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
   * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature is
   * guaranteed in this method.
   *
   * @param waypoints List of waypoints..
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  @SuppressWarnings("LocalVariableName")
  public static Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig6391 config) {
    ArrayList<Rotation2d> newHeadings = new ArrayList<>();
    for (Pose2d originalWaypoint : waypoints) {
      newHeadings.add(originalWaypoint.getRotation());
    }
    return generateTrajectory(waypoints, newHeadings, config);
  }

  /**
   * Generate spline points from a vector of splines by parameterizing the splines.
   *
   * @param splines The splines to parameterize.
   * @return The spline points for use in time parameterization of a trajectory.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines, ArrayList<Rotation2d> headings) {
    // Create the vector of spline points.
    var splinePoints = new ArrayList<PoseWithCurvature>();
    Rotation2d startHeading = new Rotation2d();

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0));

    // Store the first heading
    if (!headings.isEmpty()) {
      pointHeadings.add(headings.get(0));
      startHeading = headings.remove(0);
    }

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    for (int i = 0; i < splines.length; i++) {
      var points = SplineParameterizer.parameterize(splines[i]);

      if (!headings.isEmpty()) {
        // Get the arclength between all of the pairs of points
        var arcLengthPoints = new ArrayList<Double>();
        var firstpoint = points.remove(0);
        double arcLengthTotal = 0;
        for (final var secondpoint : points) {
          var arclength = posesCurvatureArcLength(firstpoint, secondpoint);
          arcLengthTotal += arclength;
          arcLengthPoints.add(arclength);
          firstpoint = secondpoint;
        }

        for (final var arc : arcLengthPoints) {
          pointHeadings.add(Rotation2d.fromDegrees(lerp(startHeading.getDegrees(), headings.get(i).getDegrees(), arc / arcLengthTotal)));
          startHeading = headings.get(i);
        }
      }

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size()));
    }
    return splinePoints;
  }

  public static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines) {
    return splinePointsFromSplines(splines, new ArrayList<Rotation2d>());
  }

  // Work around type erasure signatures
  @SuppressWarnings("serial")
  public static class ControlVectorList extends ArrayList<Spline.ControlVector> {
    public ControlVectorList(int initialCapacity) {
      super(initialCapacity);
    }

    public ControlVectorList() {
      super();
    }

    public ControlVectorList(Collection<? extends Spline.ControlVector> collection) {
      super(collection);
    }
  }

  /**
   The distance between the two points is a chord and the circle's radius is r (we have curvature, which is 1/r).
   The arc length for a circle is s = rθ where s is the arc length, r is the circle radius, and θ is the angle the arc spans in radians.
   This angle is the one between the two points we know.
   Let there be a perpendicular bisector between the two points that goes through the circle's center.
   This makes a right triangle whose hypotenuse is the circle's radius r, and whose opposite side from the angle we care about is chord / 2.
   We can use arcsin() (opposite over hypotenuse) to get half the arc angle, then multiply by 2 to get the whole thing.
   * @param x The first point on the arc length curve
   * @param y The second point point on the arc length curve
   * @return The total arc length between the two points given
   */
  private static double posesCurvatureArcLength(PoseWithCurvature x, PoseWithCurvature y) {
    var chord = x.poseMeters.getTranslation().getDistance(y.poseMeters.getTranslation());
    var radius = 1 / x.curvatureRadPerMeter;
    var arcangle = 2 * Math.atan((chord / 2) / radius);
    return radius * arcangle;
  }

  /**
   * Linearly interpolates between two values.
   *
   * @param startValue The start value.
   * @param endValue The end value.
   * @param t The fraction for interpolation.
   * @return The interpolated value.
   */
  @SuppressWarnings("ParameterName")
  private static double lerp(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }
}