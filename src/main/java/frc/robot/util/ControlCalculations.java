// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.FieldConstants.DISTANCE_ABOVE_FUNNEL;
import static frc.robot.Constants.FieldConstants.ROBOT_TO_TURRET_TRANSFORM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;

public class ControlCalculations {
  /**
   * Our object to hold data for shot controls
   *
   * @param shooterPitch is an angle in degrees
   * @param shooterVelocity is a speed in Inches Per Second
   */
  public record LaunchCalc(double shooterPitch, double shooterVelocity) {}

  /**
   * Calculates the distance from the robot to the target
   *
   * @param robot is the Pose2d of the robot
   * @param target is the Translation3d of the target hub
   * @return the linear distance in Meters
   */
  public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
    return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
  }

  public static Time calculateTimeOfFlight(
      Double exitVelocity, Double hoodAngle, Distance distance) {
    double vel = exitVelocity;
    double angle = hoodAngle;
    double dist = distance.in(Meters);
    return Seconds.of(dist / (vel * Math.cos(angle)));
  }

  public static Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  public static LaunchCalc SingleShot(Pose2d robot, Translation3d predictedTarget) {

    double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
    double y_dist =
        predictedTarget.getMeasureZ().minus(ROBOT_TO_TURRET_TRANSFORM.getMeasureZ()).in(Inches);
    double h = FieldConstants.FUNNEL_HEIGHT.plus(DISTANCE_ABOVE_FUNNEL).in(Inches);
    double r = FieldConstants.FUNNEL_RADIUS.in(Inches);
    double A1 = x_dist * x_dist;
    double B1 = x_dist;
    double D1 = y_dist;
    double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
    double B2 = -r;
    double D2 = h;
    double Bm = -B2 / B1;
    double A3 = Bm * A1 + A2;
    double D3 = Bm * D1 + D2;
    double a = D3 / A3;
    double b = (D1 - A1 * a) / B1;
    double theta = Math.atan(b);
    double pitchAngle = ((theta) * (180 / Math.PI));

    double g = 386; // gravity inches/sec^2
    double exitVelocity = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

    return new LaunchCalc(pitchAngle, exitVelocity);
  }

  public static LaunchCalc MultiShot(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
    LaunchCalc shot = SingleShot(robot, target);
    Distance distance = getDistanceToTarget(robot, target);
    Time timeOfFlight =
        calculateTimeOfFlight(shot.shooterVelocity(), shot.shooterPitch(), distance);
    Translation3d predictedTarget = target;

    // Iterate the process, getting better time of flight estimations and updating the predicted
    // target accordingly
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
      shot = SingleShot(robot, predictedTarget);
      timeOfFlight =
          calculateTimeOfFlight(
              shot.shooterVelocity(),
              shot.shooterPitch(),
              getDistanceToTarget(robot, predictedTarget));
    }

    return shot;
  }
}
