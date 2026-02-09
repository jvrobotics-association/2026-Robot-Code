// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.FieldConstants.DISTANCE_ABOVE_FUNNEL;
import static frc.robot.Constants.FieldConstants.ROBOT_TO_TURRET_TRANSFORM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;

public class ControlCalculations {

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

  public static Angle HoodAngle(Pose2d robot, Translation3d predictedTarget) {

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
    return Radians.of(theta);
  }
}
