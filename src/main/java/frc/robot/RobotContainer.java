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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.ExtenderCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.end_effector_wheels.EndEffectorWheels;
import frc.robot.subsystems.end_effector_wheels.FlywheelIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.sensor.CoralSensorIO;
import frc.robot.subsystems.intake.sensor.CoralSensorIORoboRio;
import frc.robot.subsystems.intake.sensor.CoralSensorIOSim;
import frc.robot.subsystems.intake.wheel.IntakeWheelIO;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOSim;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOSparkMax;
import frc.robot.subsystems.poseEstimator.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final IntakeSubsystem intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                new Vision());
        intake = 
            new IntakeSubsystem(
                new IntakeWheelIOSparkMax(1, 0), 
                new CoralSensorIORoboRio());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                new Vision());
        intake = 
            new IntakeSubsystem(
                new IntakeWheelIOSim(),
                 new CoralSensorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new Vision());
        intake = 
            new IntakeSubsystem(
                new IntakeWheelIO() {},
                new CoralSensorIO() {});
        break;
    }

    // Register named commands

    NamedCommands.registerCommand(
        "Extender to L1", ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L1, 2));
    NamedCommands.registerCommand(
        "Extender to L2", ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L2, 2));
    NamedCommands.registerCommand(
        "Extender to L3", ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L3, 2));
    NamedCommands.registerCommand(
        "Extender to L4", ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L4, 2));
    NamedCommands.registerCommand(
        "Extender to Intake",
        ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.Intake, 2));
    NamedCommands.registerCommand(
        "Extender to Process",
        ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.Process, 2));
    NamedCommands.registerCommand(
        "Extender to L2 Dealgify",
        ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L2Dealgify, 2));
    NamedCommands.registerCommand(
        "Extender to L3 Dealgify",
        ExtenderCommands.goToSetpoint(ExtenderCommands.ExtenderSetpoint.L3Dealgify, 2));

    NamedCommands.registerCommand(
        "Intake Coral From Station",
        IntakeCommands.intakeCoralFromStation(intake, ExtenderCommands.ExtenderSetpoint.Intake));

    NamedCommands.registerCommand("Score Coral", EndEffectorCommands.scoreCoral());
    NamedCommands.registerCommand("Dealgify", EndEffectorCommands.dealgify());

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

    autoChooser.addOption(
        "Move Test", Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0))));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // if (vision.returnTargets(vision.camera) != null) {
    //   vision.returnBestPose();
    // }

    var hid = controller.getHID();
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> hid.getLeftY(),
            () -> hid.getLeftX(),
            () -> -hid.getRightX(),
            () -> (Math.abs(hid.getRightX()) > 0.1)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    var flywheel =
        new EndEffectorWheels(
            new FlywheelIOSim(new DCMotor(100.0, 10.0, 10.0, 10.0, 100, 1), 1, 1));

    controller.y().whileTrue(Commands.run(() -> flywheel.setVelocity(10)));
  }

  public void update() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
