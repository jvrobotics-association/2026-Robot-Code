// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoDriveWithAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Tower;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Indexer indexer;
  private final Tower tower;
  private final Intake intake;
  private final IntakeExtension intakeExt;
  private final Hopper hopper;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick operatorPanel = new CommandJoystick(1);

  private Translation3d hubTarget;
  private Command hopperExtendCommand;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        shooter = new Shooter();
        pitch = new ShooterPitch();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        shooter = new Shooter();
        pitch = new ShooterPitch();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter();
        pitch = new ShooterPitch();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();

        // Pass nulls or empty shells for Replay mode depending on your AdvantageKit setup
        // midSystem = new MidSystem(null, null, null, null, null, null, null, null, null, null);
        break;
    }

    // Create the named commands for PathPlanner Autos
    NamedCommands.registerCommand(
        "runIntake", Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    NamedCommands.registerCommand(
        "raiseIntakeArm",
        Commands.sequence(
            Commands.runOnce(intakeExt::retract, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(intakeExt::deploy, intakeExt)));
    NamedCommands.registerCommand(
        "runShooter",
        Commands.parallel(
                Commands.run(pitch::moveToPosition, pitch),
                Commands.run(shooter::shoot, shooter),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.parallel(
                        Commands.startEnd(indexer::feed, indexer::stop, indexer),
                        Commands.runEnd(() -> tower.setManualDutyCycle(0.8), tower::stop, tower))))
            .finallyDo(
                () -> {
                  pitch.movetoMinPosition();
                  shooter.stop();
                }));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    configureBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureBindings() {
    /////////////////////////////////
    //////// COMMON COMMANDS ////////
    /////////////////////////////////

    // Runs the intake to intake fuel
    Command runIntakeCommand = Commands.startEnd(intake::startIntake, intake::stopIntake, intake);

    // Raises the intake arm and lower it back down to help feed fuel to the shooter
    Command raiseIntakeArmCommand =
        Commands.sequence(
                Commands.runOnce(intakeExt::retract, intakeExt),
                Commands.runOnce(intake::startIntake, intake),
                Commands.waitSeconds(1),
                Commands.runOnce(intakeExt::deploy, intakeExt))
            .finallyDo(() -> intake.stopIntake());

    // Spins up the shooter, then runs the indexer and tower to feed fuel to the shooter
    Command shootCommand =
        Commands.parallel(
                Commands.run(pitch::moveToPosition, pitch),
                Commands.run(shooter::shoot, shooter),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.parallel(
                        Commands.startEnd(indexer::feed, indexer::stop, indexer),
                        Commands.runEnd(() -> tower.setManualDutyCycle(0.8), tower::stop, tower))))
            .finallyDo(
                () -> {
                  pitch.movetoMinPosition();
                  shooter.stop();
                });

    // Retracts the intake arm and hopper
    Command hopperRetractCommand =
        Commands.sequence(
            Commands.runOnce(intakeExt::retract, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(intakeExt::stop, intakeExt),
            Commands.runOnce(hopper::retract, hopper),
            Commands.waitSeconds(5),
            Commands.runOnce(hopper::stop, hopper));

    // Extends the hopper and intake arm
    hopperExtendCommand =
        Commands.sequence(
            Commands.runOnce(hopper::deploy, hopper),
            Commands.waitSeconds(1),
            Commands.runOnce(intakeExt::deploy, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(intakeExt::stop, intakeExt));

    // Locks the wheels to an X pattern to make the robot harder to move
    Command xLockWheelsCommand = Commands.runOnce(drive::stopWithX, drive);

    ///////////////////////////////////
    //////// CONTROLLER INPUTS ////////
    ///////////////////////////////////

    // Default for the drive command to field oriented drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Run the intake
    controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD).whileTrue(runIntakeCommand);

    // Automatically follow a path or pathfind to a specific pose to align for shooting. Locks the
    // wheels to the X pattern once at destination
    controller.x().whileTrue(Commands.sequence(new AutoDriveWithAlign(drive), xLockWheelsCommand));

    // Aim the robot at the hub, this does not follow a path and simply aims the bot while allowing
    // for X & Y travel
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    new Rotation2d(
                        hubTarget.getX() - drive.getPose().getX(),
                        hubTarget.getY() - drive.getPose().getY())));

    // Shoot the balls once the robot is aligned
    controller.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD).whileTrue(shootCommand);


    // Manually pull the hopper back in when the zeroing is incorrect
    controller
        .button(7)
        .whileTrue(Commands.runEnd(() -> hopper.setManualDutyCycle(-0.1), hopper::stop, hopper));

    // Resets the robots Zero heading to the current orientation of the robot
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    ///////////////////////////////////////
    //////// OPERATOR PANEL INPUTS ////////
    ///////////////////////////////////////

    // Manually cycle the hopper/intake arm to retract
    operatorPanel.button(13).onTrue(hopperRetractCommand);

    // Manually cycle the hopper/intake arm to extend
    operatorPanel.button(6).onTrue(hopperExtendCommand);

    // Run the intake
    operatorPanel.button(4).whileTrue(runIntakeCommand);

    // Manually outake fuel by running the indexer and intake in reverse
    operatorPanel
        .button(12)
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> intake.setManualDutyCycle(-0.5), intake::stopIntake, intake),
                Commands.startEnd(() -> indexer.setManualDutyCycle(-20), indexer::stop, indexer)));

    // Raise the intake arm so that balls in the front of the hopper are moved to the back
    operatorPanel.button(2).onTrue(raiseIntakeArmCommand);

    // Shoot the balls once the robot is aligned
    operatorPanel.button(1).whileTrue(shootCommand);

    // Lock the drive modules to an X configuration to help avoid getting bumped around
    // operatorPanel.button(14).onTrue(xLockWheelsCommand);
  }

  /**
   * Triggers a refresh of the hub target by checking which alliance we are on and setting the
   * appropriate location
   */
  public void updateHubTarget() {
    hubTarget =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;
    ;
  }

  /** Schedules the hopperExtendCommand to extend the hopper and intake arm */
  public void extendHopper() {
    CommandScheduler.getInstance().schedule(hopperExtendCommand);
  }
}
