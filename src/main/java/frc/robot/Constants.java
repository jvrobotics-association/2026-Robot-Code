// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class ClimberConstants {
    public static final int MOTOR_ID = 60;

    public static final double STATOR_AMP_LIMIT = 40;
    public static final double PEAK_FORWARD_TORQUE_CURRENT = 40;
    public static final double PEAK_REVERSE_TORQUE_CURRENT = -40;
    public static final double SENSOR_TO_MECH_RATIO = 48;
    public static final double MM_CRUISE_VEL = 0;
    public static final double MM_ACCELERATION = 0;
    public static final double PID_KP = 0;
    public static final double PID_KD = 0;
    public static final double PID_KS = 0;
    public static final double PID_KV = 0;
    public static final double TOLERANCE_ROTATIONS = 0;
    public static final double CLIMB_ROTATIONS = 0;
    public static final double PREPARE_ROTATIONS = 0;
  }

  public final class IndexerConstants {
    public static final int CAN_ID = 22;
    public static final double INDEXER_SPEED = 0.4;
  }

  public final class ShooterConstants {
    public final class RightMotor {
      public static final int MOTOR_ID = 30;
      public static final double SENSOR_TO_MECH = 0.75;
      public static final int SUPPLY_CURRENT_LIMIT = 45;
      public static final int SUPPLY_CURRENT_LOWER_LIMIT = 29;
      public static final int SUPPLY_CURRENT_LOWER_TIME = 2;
      public static final int PEAK_FORWARD_VOLTAGE = 12;
      public static final int PEAK_REVERSE_VOLTAGE = -12;
      public static final double PID_KS = 1.2;
      public static final double PID_KV = 0.5;
      public static final double PID_KP = 6;
      public static final double MM_ACCELERATION = 200;
    }

    public final class LeftMotor {
      public static final int MOTOR_ID = 40;
      public static final double SENSOR_TO_MECH = 0.75;
      public static final int SUPPLY_CURRENT_LIMIT = 45;
      public static final int SUPPLY_CURRENT_LOWER_LIMIT = 29;
      public static final int SUPPLY_CURRENT_LOWER_TIME = 2;
      public static final int PEAK_FORWARD_VOLTAGE = 12;
      public static final int PEAK_REVERSE_VOLTAGE = -12;
      public static final double PID_KS = 1.2;
      public static final double PID_KV = 0.5;
      public static final double PID_KP = 6;
      public static final double MM_ACCELERATION = 200;
    }
  }

  public final class ShooterPitchConstants {
    public static final int MOTOR_ID = 31;
    public static final int SUPPLY_CURRENT_LIMIT = 15;
    public static final int SUPPLY_CURRENT_LOWER_LIMIT = 9;
    public static final int SUPPLY_CURRENT_LOWER_TIME = 1;
    public static final int PEAK_FORWARD_VOLTAGE = 12;
    public static final int PEAK_REVERSE_VOLTAGE = -12;
    public static final double MIN_ROTATION = 0.0;
    public static final double MAX_ROTATION = 0.0568850;
    public static final double PID_KP = 27;
    public static final double PID_KD = 0.1;
    public static final double PID_KS = 0.27;
    public static final double PID_KV = 0.1;
    public static final double MM_ACCELERATION = 15;
    public static final double MM_CRUISE_VEL = 5;
  }

  public final class IntakeConstants {
    public static final double INTAKE_SPEED = 85;

    public static class LeftMotor {
      public static final int MOTOR_ID = 23;
      public static final double SENSOR_TO_MECH_RATIO = 1.667;
      public static final int SUPPLY_CURRENT_LIMIT = 50;
      public static final int SUPPLY_CURRENT_LOWER_LIMIT = 30;
      public static final double SUPPLY_CURRENT_LOWER_TIME = 2;
      public static final int PEAK_FORWARD_VOLTAGE = 12;
      public static final int PEAK_REVERSE_VOLTAGE = -12;
      public static final double PID_KS = 0.37;
      public static final double PID_KV = 0.17;
      public static final double PID_KP = 14.0;
      public static final double MM_ACCELERATION = 150;
    }

    public static class RightMotor {
      public static final int MOTOR_ID = 20;
      public static final double SENSOR_TO_MECH_RATIO = 1.667;
      public static final int SUPPLY_CURRENT_LIMIT = 50;
      public static final int SUPPLY_CURRENT_LOWER_LIMIT = 30;
      public static final double SUPPLY_CURRENT_LOWER_TIME = 2;
      public static final int PEAK_FORWARD_VOLTAGE = 12;
      public static final int PEAK_REVERSE_VOLTAGE = -12;
      public static final double PID_KS = 0.37;
      public static final double PID_KV = 0.17;
      public static final double PID_KP = 14.0;
      public static final double MM_ACCELERATION = 150;
    }
  }

  public final class IntakeExtensionConstants {
    public static class ExtensionMotor {
      public static final int MOTOR_ID = 21;
      public static final double ROTOR_TO_SENSOR_RATIO = 45;
      public static final int SUPPLY_CURRENT_LIMIT = 45;
      public static final int SUPPLY_CURRENT_LOWER_LIMIT = 29;
      public static final double SUPPLY_CURRENT_LOWER_TIME = 2;
      public static final int PEAK_FORWARD_VOLTAGE = 12;
      public static final int PEAK_REVERSE_VOLTAGE = -12;
      public static final double MIN_ROTATION = 0;
      public static final double MAX_ROTATION = 0.35;
      public static final double PID_KP = 200;
      public static final double PID_KD = 55;
      public static final double PID_KS = 0.5;
      public static final double PID_KV = 0.1;
      public static final double PID_KG = -0.6;
      public static final double PID_ARM_OFFSET = -0.147;
      public static final double MM_CRUISE_VEL = 1;
      public static final double MM_ACCELERATION = 3;
    }

    public static class ExtensionEncoder {
      public static final int SENSOR_ID = 8;
      public static final double MAGNET_OFFSET = -0.11;
      public static final double ABSOLUTE_SENSOR_DISCONTINUITY = 0.5;
    }

    public static final double TOLERANCE_ROTATIONS = 0.003;
    public static final double RETRACTED_ROTATIONS = 0.003;
    public static final double DEPLOYED_ROTATIONS = 0.3525;
  }

  public final class TowerConstants {
    public static final int MOTOR_ID = 3;
    public static final double TOWER_SPEED = 0.8;
    public static final int SUPPLY_CURRENT_LIMIT = 45;
    public static final int SUPPLY_CURRENT_LOWER_LIMIT = 29;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 2;
    public static final int PEAK_FORWARD_VOLTAGE = 12;
    public static final int PEAK_REVERSE_VOLTAGE = -12;
  }

  public final class HopperConstants {
    public static final int MOTOR_ID = 2;
    public static final double SENSOR_TO_MECH_RATIO = 30.4;
    public static final int SUPPLY_CURRENT_LIMIT = 15;
    public static final int SUPPLY_CURRENT_LOWER_LIMIT = 10;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 2;
    public static final int PEAK_FORWARD_VOLTAGE = 12;
    public static final int PEAK_REVERSE_VOLTAGE = -12;
    public static final double PID_KP = 244;
    public static final double PID_KD = 15;
    public static final double PID_KS = 0.49;
    public static final double PID_KV = 2.9;
    public static final double MM_CRUISE_VEL = 16;
    public static final double MM_ACCELERATION = 50;
    public static final double DEPLOYED_ROTATIONS = 5.4;
    public static final double RETRACTED_ROTATIONS = 0.01;
    public static final double TOLERANCE_ROTATIONS = 0.005;
  }

  public final class RobotConstants {
    public static final Distance FULL_WIDTH = Inches.of(25); // TODO: Verify Frame plus bumper
    public static final Distance FULL_LENGTH = Inches.of(29.5); // TODO: Verify Frame plus bumper
  }

  public final class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(651.22);
    public static final Distance FIELD_WIDTH = Inches.of(317.69);
    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static final Translation3d HUB_BLUE =
        new Translation3d(Inches.of(182.11), FIELD_WIDTH.div(2), Inches.of(56.44));
    public static final Translation3d HUB_RED =
        new Translation3d(
            FIELD_LENGTH.minus(Inches.of(182.11)), FIELD_WIDTH.div(2), Inches.of(56.44));

    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);
    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.zero(), Inches.of(-6), Inches.of(20.625)), Rotation3d.kZero);
    public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(6);

    public static final Pose2d LEFT_SHOOT_POS_BLUE =
        new Pose2d(
            Distance.ofBaseUnits(2.544, Meters),
            Distance.ofBaseUnits(5.665, Meters),
            new Rotation2d(Angle.ofRelativeUnits(-42, Degree)));

    public static final Pose2d RIGHT_SHOOT_POS_BLUE =
        new Pose2d(
            Distance.ofBaseUnits(2.544, Meters),
            Distance.ofBaseUnits(2.403, Meters),
            new Rotation2d(Angle.ofRelativeUnits(42, Degree)));

    public static final Pose2d LEFT_SHOOT_POS_RED =
        new Pose2d(
            Distance.ofBaseUnits(13.996, Meters),
            Distance.ofBaseUnits(2.403, Meters),
            new Rotation2d(Angle.ofRelativeUnits(138, Degree)));

    public static final Pose2d RIGHT_SHOOT_POS_RED =
        new Pose2d(
            Distance.ofBaseUnits(13.996, Meters),
            Distance.ofBaseUnits(5.665, Meters),
            new Rotation2d(Angle.ofRelativeUnits(-138, Degree)));
  }

  public final class ControllerConstants {
    public static final double TRIGGER_THRESHOLD = 0.25;
  }
}
