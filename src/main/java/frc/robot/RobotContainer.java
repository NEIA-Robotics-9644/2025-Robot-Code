// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlignCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.end_effector.EndEffectorSubsystem;
import frc.robot.subsystems.end_effector.FlywheelIOSim;
import frc.robot.subsystems.end_effector.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.sensor.CoralSensorIOSim;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOSim;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOSparkMax;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
import frc.robot.subsystems.poseEstimator.Vision;
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
  private final Intake intakeWheels;
  private final EndEffectorSubsystem endEffectorWheels;
  private final Elevator elevator;
  private final Pivot pivot;

  // Driver controller
  private final CommandXboxController driveCon = new CommandXboxController(0);

  // Operator controller
  private final CommandXboxController opCon = new CommandXboxController(1);

  ControllerState controllerState = new ControllerState();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new ModuleIOSim(TunerConstants.BackRight),
                new Vision());

        endEffectorWheels = new EndEffectorSubsystem(new FlywheelIOSim());
        elevator = new Elevator(new ElevatorIOSim(), new LimitSwitchSensorIOSim());
        pivot = new Pivot(new PivotIOSim());
        intakeWheels = new Intake(new IntakeWheelIOSim(), new CoralSensorIOSim());
        break;
      default:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                new Vision());
        endEffectorWheels = new EndEffectorSubsystem(new FlywheelIOSparkMax(23));
        elevator =
            new Elevator(
                new ElevatorIOSparkMax(20, 21, false, true),
                new LimitSwitchSensorIORoboRio(9, true));
        pivot = new Pivot(new PivotIOSparkMax(22));

        intakeWheels = new Intake(new IntakeWheelIOSparkMax(24, 1, 40), new CoralSensorIOSim());

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    var moveForwardAuto =
        DriveCommands.driveChassisSpeeds(drive, () -> new ChassisSpeeds(1, 0, 0)).withTimeout(8);

    var scoreL1Auto =
        AutoCommands.manualScore(drive, elevator, pivot, endEffectorWheels, 0.2, 0.1);

    var scoreL2Auto =
        AutoCommands.manualScore(drive, elevator, pivot, endEffectorWheels, 0.286, 0.48);

    var scoreL3Auto =
        AutoCommands.manualScore(drive, elevator, pivot, endEffectorWheels, 0.55, 0.48);

    var scoreL4Auto =
        AutoCommands.manualScore(drive, elevator, pivot, endEffectorWheels, 1, 0.81);

    autoChooser.addOption("Move forward", moveForwardAuto);

    autoChooser.addOption("Score L1", scoreL1Auto);

    autoChooser.addOption("Score L2", scoreL2Auto);

    autoChooser.addOption("Score L3", scoreL3Auto);

    autoChooser.addOption("Score L4", scoreL4Auto);

    // Configure the button bindings
    configureButtonBindings();
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
    // Default command, normal field-relative drive
    Command driveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -hid.getLeftY() * controllerState.getCurrentDriveSpeed().translationScale,
            () -> -hid.getLeftX() * controllerState.getCurrentDriveSpeed().translationScale,
            () -> -hid.getRightX() * controllerState.getCurrentDriveSpeed().rotationScale * 0.65,
            () -> driveCon.povRight().getAsBoolean());

    driveCommand.addRequirements(drive);
    drive.setDefaultCommand(driveCommand);
    driveCon.rightTrigger(0.5).whileTrue(AutoAlignCommands.closestReefAlign(drive));

    driveCon.leftBumper().onTrue(Commands.runOnce(() -> controllerState.decreaseDriveSpeedIndex()));
    driveCon
        .rightBumper()
        .onTrue(Commands.runOnce(() -> controllerState.increaseDriveSpeedIndex()));

    // --- Operator Controls ---
    opCon
        .rightTrigger(0.1)
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

    opCon
        .leftTrigger(0.1)
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
    new Trigger(DriverStation::isTeleopEnabled).onTrue(elevator.home());

    elevator.setDefaultCommand(
        elevator.goToHeight(() -> controllerState.getCurrentSetpoint().height));

    pivot.setDefaultCommand(pivot.goToAngle(() -> controllerState.getCurrentSetpoint().angle));

    // When the left bumper is held, manually control the elevator and pivot with the joysticks

    opCon
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                elevator.manualControl(opCon::getLeftY), pivot.manualControl(opCon::getRightY)));

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
        .onTrue(Commands.runOnce(() -> controllerState.getCurrentSetpoint().height += 0.01));
    opCon
        .povDown()
        .onTrue(Commands.runOnce(() -> controllerState.getCurrentSetpoint().height -= 0.01));

    opCon
        .povLeft()
        .onTrue(Commands.runOnce(() -> controllerState.getCurrentSetpoint().angle -= 0.01));
    opCon
        .povRight()
        .onTrue(Commands.runOnce(() -> controllerState.getCurrentSetpoint().angle += 0.01));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll();
    return autoChooser.get();
  }
}
