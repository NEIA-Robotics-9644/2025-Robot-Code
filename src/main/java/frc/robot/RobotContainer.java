package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExtenderCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.elevator_motors.ElevatorIOSim;
import frc.robot.subsystems.elevator.elevator_motors.ElevatorIOSparkMax;
import frc.robot.subsystems.elevator.limit_sensor.LimitSwitchSensorIORoboRio;
import frc.robot.subsystems.elevator.limit_sensor.LimitSwitchSensorIOSim;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.FlywheelIOSim;
import frc.robot.subsystems.end_effector.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeWheelIOSim;
import frc.robot.subsystems.intake.IntakeWheelIOSparkMax;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ExtenderConstraints;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intakeWheels;
  private final EndEffector endEffectorWheels;
  private final Elevator elevator;
  private final Pivot pivot;
  private final Climber climber;

  // Driver controller
  private final CommandXboxController driveCon = new CommandXboxController(0);

  // Operator controller
  private final CommandXboxController opCon = new CommandXboxController(1);

  ControllerState controllerState = new ControllerState();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private LoggedTunableNumber alignPredictionSeconds =
      new LoggedTunableNumber("Align Prediction Seconds", 0.3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Real robot, instantiate hardware IO implementations

    switch (Constants.currentMode) {
      case SIM:
        // Sim robot, instantiate sim hardware IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight)
                // new Vision()
                );
        vision =
            new Vision(
                drive,
                drive,
                new VisionIO[] {
                  new VisionIOPhotonVisionSim(
                      VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(
                      VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                  new VisionIOPhotonVisionSim(
                      VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose)
                });

        endEffectorWheels = new EndEffector(new FlywheelIOSim());
        elevator = new Elevator(new ElevatorIOSim(), new LimitSwitchSensorIOSim());
        pivot = new Pivot(new PivotIOSim());
        intakeWheels = new Intake(new IntakeWheelIOSim());
        climber = new Climber(new ClimberIOSim(), 9, 7);
        break;
      default:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight)
                // new Vision()
                );

        vision =
            new Vision(
                drive,
                drive,
                new VisionIO[] {
                  new VisionIOPhotonVision(
                      VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                  new VisionIOPhotonVision(
                      VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                  new VisionIOPhotonVision(
                      VisionConstants.camera2Name, VisionConstants.robotToCamera2)
                });

        endEffectorWheels = new EndEffector(new FlywheelIOSparkMax(23));
        elevator =
            new Elevator(
                new ElevatorIOSparkMax(20, 21, false, true),
                new LimitSwitchSensorIORoboRio(9, true));
        pivot = new Pivot(new PivotIOSparkMax(22));
        climber = new Climber(new ClimberIOSparkMax(25), 9, 7);

        intakeWheels = new Intake(new IntakeWheelIOSparkMax(24, 1, 40));

        break;
    }

    NamedCommands.registerCommand("L1", controllerState.setSetpoint(controllerState.L1));

    NamedCommands.registerCommand("L3", controllerState.setSetpoint(controllerState.L3));

    // NamedCommands.registerCommand(
    //  "L4", new ParallelCommandGroup(elevator.goToHeight(() -> 1), pivot.goToAngle(() -> 0.81)));
    NamedCommands.registerCommand("L4", controllerState.setSetpoint(controllerState.L4));

    NamedCommands.registerCommand("Stow", controllerState.setSetpoint(controllerState.INTAKE));

    NamedCommands.registerCommand(
        "AutoAlignRight",
        DriveCommands.joystickApproach(
            drive,
            () -> 0.5,
            () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));
    NamedCommands.registerCommand(
        "AutoAlignLeft",
        DriveCommands.joystickApproach(
            drive,
            () -> 0.5,
            () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

    NamedCommands.registerCommand(
        "AutoScoreLeft",
        AutoCommands.autoScore(false, ReefSide.LEFT, drive, endEffectorWheels, controllerState));
    NamedCommands.registerCommand(
        "AutoScoreRight",
        AutoCommands.autoScore(false, ReefSide.RIGHT, drive, endEffectorWheels, controllerState));

    NamedCommands.registerCommand(
        "SlowAutoScoreLeft",
        AutoCommands.autoScore(true, ReefSide.LEFT, drive, endEffectorWheels, controllerState));
    NamedCommands.registerCommand(
        "SlowAutoScoreRight",
        AutoCommands.autoScore(true, ReefSide.RIGHT, drive, endEffectorWheels, controllerState));

    NamedCommands.registerCommand(
        "Score",
        Commands.startEnd(
                () -> endEffectorWheels.setVelocity(1), () -> endEffectorWheels.setVelocity(0))
            .withTimeout(0.75));

    NamedCommands.registerCommand(
        "Intake",
        Commands.startEnd(
            () -> {
              intakeWheels.setVelocity(-0.5);
              endEffectorWheels.setVelocity(0.5);
            },
            () -> {
              intakeWheels.setVelocity(0);
              endEffectorWheels.setVelocity(0);
            }));

    NamedCommands.registerCommand(
        "Unjam",
        Commands.startEnd(
            () -> {
              intakeWheels.setVelocity(0.5);
              endEffectorWheels.setVelocity(-0.5);
            },
            () -> {
              intakeWheels.setVelocity(0);
              endEffectorWheels.setVelocity(0);
            }));

    NamedCommands.registerCommand("Test", new PrintCommand("TESTING AUTO COMMAND"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    autoChooser.addOption("Three Piece Left", new PathPlannerAuto("Three Piece Left", false));

    autoChooser.addOption(
        "Slow Three Piece Left", new PathPlannerAuto("Slow Three Piece Left", false));
    autoChooser.addOption(
        "Three Piece Right", new PathPlannerAuto("Three Piece Right Unmirrored", true));

    autoChooser.addOption(
        "Three Piece Right Unique", new PathPlannerAuto("Three Piece Right Unique", false));
    autoChooser.addOption("Two Piece Left", new PathPlannerAuto("Two Piece Left", false));

    autoChooser.addOption("One Piece Left", new PathPlannerAuto("One Piece Left", false));
    autoChooser.addOption(
        "One Piece Right", new PathPlannerAuto("One Piece Right Unmirrored", true));

    autoChooser.addOption(
        "Delayed One Piece Left", new PathPlannerAuto("Delayed One Piece Left", false));
    autoChooser.addOption(
        "Delayed One Piece Right", new PathPlannerAuto("Delayed One Piece Right Unmirrored", true));

    autoChooser.addOption(
        "Drive Forward", Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0))));

    autoChooser.addOption("Center Left", new PathPlannerAuto("One Piece Left"));

    // Configure the button bindings
    configureButtonBindings();
  }

  private Command joystickApproach(Supplier<Pose2d> approachPose) {
    return DriveCommands.joystickApproach(
        drive, () -> -driveCon.getHID().getLeftY() * 0.9, approachPose);
  }

  private Command L1joystickApproach(Supplier<Pose2d> approachPose) {
    return DriveCommands.joystickApproach(drive, () -> 0.01, approachPose);
  }

  private Pose2d getFuturePose(double seconds) {
    return drive.getPose().exp(drive.getChassisSpeeds().toTwist2d(seconds));
  }

  public Command spitAndStrafe(ReefSide side) {
    return Commands.deadline(
        AutoCommands.autoL1(controllerState, endEffectorWheels),
        /*Commands.either(
        L1joystickApproach(
            () ->
                FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), side)
                    .transformBy(
                        new Transform2d(0, Units.inchesToMeters(24), new Rotation2d()))),
        L1joystickApproach(
            () ->
                FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), side)
                    .transformBy(
                        new Transform2d(0, Units.inchesToMeters(-24), new Rotation2d()))),
        () -> side == ReefSide.RIGHT)); */

        Commands.either(
            L1joystickApproach(
                () ->
                    FieldConstants.getNearestReefFace(drive.getPose())
                        .transformBy(
                            new Transform2d(0, Units.inchesToMeters(3), new Rotation2d()))),
            L1joystickApproach(
                () ->
                    FieldConstants.getNearestReefFace(drive.getPose())
                        .transformBy(
                            new Transform2d(0, Units.inchesToMeters(-3), new Rotation2d()))),
            () -> side == ReefSide.RIGHT));
  }

  public void onTeleopEnable() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // --- Driver Controls ---

    var hid = driveCon.getHID();
    var opHid = opCon.getHID();
    // Default command, normal field-relative drive
    Command driveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -(hid.getLeftY()) * controllerState.getCurrentDriveSpeed().translationScale,
            () -> -(hid.getLeftX()) * controllerState.getCurrentDriveSpeed().translationScale,
            () -> (hid.getRightX()) * controllerState.getCurrentDriveSpeed().rotationScale * 0.65,
            () -> driveCon.povRight().getAsBoolean());

    driveCommand.addRequirements(drive);
    drive.setDefaultCommand(driveCommand);
    // driveCon
    //     .rightTrigger(0.5)
    //     .whileTrue(
    //         ReefTagAlignCommand.reefTagAlign(
    //             drive,
    //             vision,
    //             () -> driveCon.getLeftY() * 0.63,
    //             ReefTagAlignCommand.AlignGoal.RIGHT));

    // driveCon
    //     .leftTrigger(0.5)
    //     .whileTrue(
    //         ReefTagAlignCommand.reefTagAlign(
    //             drive,
    //             vision,
    //             () -> driveCon.getLeftY() * 0.63,
    //             ReefTagAlignCommand.AlignGoal.LEFT));

    // driveCon
    //     .y()
    //     .whileTrue(
    //         ReefTagAlignCommand.reefTagAlign(
    //             drive,
    //             vision,
    //             () -> driveCon.getLeftY() * 0.63,
    //             ReefTagAlignCommand.AlignGoal.CENTER));

    driveCon
        .rightTrigger(0.5)
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));
    driveCon
        .leftTrigger(0.5)
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));
    driveCon
        .y()
        .whileTrue(joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose())));

    driveCon.leftBumper().and(driveCon.a()).whileTrue(spitAndStrafe(ReefSide.LEFT));

    driveCon.rightBumper().and(driveCon.a()).whileTrue(spitAndStrafe(ReefSide.RIGHT));

    // Drop is left d pad plus burger button
    var drop = new Trigger(() -> driveCon.getHID().getPOV() == 270).and(driveCon.start());

    drop.onTrue(Commands.runOnce(() -> climber.toggleIntake()));

    var lock = driveCon.start().and(driveCon.back());

    lock.onTrue(Commands.runOnce(() -> climber.toggleClimbLock()));

    var extenderConstraints = new ExtenderConstraints("constraints.txt", 80);

    climber.setDefaultCommand(
        climber.moveWithVelocity(
            () -> {
              if (driveCon.getHID().getPOV() == 0) {
                return 20;
              } else if (driveCon.getHID().getPOV() == 180) {
                return -20;
              } else {
                return 0;
              }
            }));

    // Lock is screenshare button plus burger button

    driveCon.leftBumper().onTrue(Commands.runOnce(() -> controllerState.decreaseDriveSpeedIndex()));

    driveCon
        .rightBumper()
        .onTrue(Commands.runOnce(() -> controllerState.increaseDriveSpeedIndex()));

    driveCon
        .x()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intakeWheels.setVelocity(0.75 * -0.5);
                  endEffectorWheels.setVelocity(0.75 * 0.5);
                },
                () -> {
                  intakeWheels.setVelocity(0);
                  endEffectorWheels.setVelocity(0);
                }));

    // --- Operator Controls ---

    opCon
        .rightTrigger(0.05)
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intakeWheels.setVelocity(opCon.getRightTriggerAxis() * -0.5);
                  endEffectorWheels.setVelocity(opCon.getRightTriggerAxis() * 0.5);
                },
                () -> {
                  intakeWheels.setVelocity(0);
                  endEffectorWheels.setVelocity(0);
                }));

    // new Trigger(DriverStation::isEnabled).onTrue(elevator.home());

    elevator.setDefaultCommand(
        ExtenderCommands.goToHeightThenPivot(
            elevator,
            pivot,
            extenderConstraints,
            () -> controllerState.getCurrentSetpoint().inchesFromGround,
            () -> controllerState.getCurrentSetpoint().degreesFromVertical));

    opCon
        .leftTrigger(0.05)
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intakeWheels.setVelocity(opCon.getLeftTriggerAxis() * 0.5);
                  endEffectorWheels.setVelocity(opCon.getLeftTriggerAxis() * -0.5);
                },
                () -> {
                  intakeWheels.setVelocity(0);
                  endEffectorWheels.setVelocity(0);
                }));

    // When the robot is enabled, go into homing mode

    opCon
        .leftBumper()
        .whileTrue(
            controllerState.runManualSetpoint(
                elevator, pivot, () -> -opCon.getLeftY() * 1, () -> opCon.getRightX() * 1));

    // When the right bumper is pressed, go to Intake setpoint
    opCon
        .rightBumper()
        .onTrue(Commands.runOnce(() -> controllerState.setCurrentSetpoint(controllerState.INTAKE)));

    // When the A button is pressed, go to L1
    opCon
        .a()
        .onTrue(Commands.runOnce(() -> controllerState.setCurrentSetpoint(controllerState.L1)));

    // When the B button is pressed, go to L2
    opCon
        .b()
        .onTrue(Commands.runOnce(() -> controllerState.setCurrentSetpoint(controllerState.L2)));

    // When the X button is pressed, go to L3
    opCon
        .x()
        .onTrue(Commands.runOnce(() -> controllerState.setCurrentSetpoint(controllerState.L3)));

    // When the Y button is pressed, go to L4
    opCon
        .y()
        .onTrue(Commands.runOnce(() -> controllerState.setCurrentSetpoint(controllerState.L4)));

    opCon
        .start()
        .onTrue(
            Commands.runOnce(
                () -> controllerState.setCurrentSetpoint(controllerState.LowDealgify)));

    new Trigger(() -> opCon.getHID().getBackButtonPressed())
        .onTrue(
            Commands.runOnce(
                () -> controllerState.setCurrentSetpoint(controllerState.HighDealgify)));

    opCon
        .povUp()
        .onTrue(
            Commands.runOnce(() -> controllerState.getCurrentSetpoint().inchesFromGround += 0.5));
    opCon
        .povDown()
        .onTrue(
            Commands.runOnce(() -> controllerState.getCurrentSetpoint().inchesFromGround -= 0.5));

    opCon
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> controllerState.getCurrentSetpoint().degreesFromVertical -= 1));
    opCon
        .povRight()
        .onTrue(
            Commands.runOnce(() -> controllerState.getCurrentSetpoint().degreesFromVertical += 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Cancel all running commands
    return autoChooser.get();
  }
}
