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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.ExtenderCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extender.ExtenderSubsystem;
import frc.robot.subsystems.extender.elevator.ElevatorIO.ElevatorIOConfig;
import frc.robot.subsystems.extender.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.extender.pivot.PivotIOSparkMax;
import frc.robot.subsystems.extender.sensor.LimitSwitchSensorIORoboRio;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.sensor.CoralSensorIOSim;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOSparkMax;
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
  private final IntakeSubsystem intakeWheels;
  private final IntakeSubsystem endEffectorWheels;
  private final ExtenderSubsystem extender;

  // Driver controller
  private final CommandXboxController driveCon = new CommandXboxController(0);

  // Operator controller
  private final CommandXboxController opCon = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    var elevatorIOConfig = new ElevatorIOConfig(0.3, 0.01, 0.12, 0.0008, 1, 100.0, 60, false, true);

    // Real robot, instantiate hardware IO implementations
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight),
            new Vision());
    endEffectorWheels =
        new IntakeSubsystem(new IntakeWheelIOSparkMax(23, 1, 40), new CoralSensorIOSim());
    extender =
        new ExtenderSubsystem(
            new ElevatorIOSparkMax(20, 21, elevatorIOConfig),
            new PivotIOSparkMax(22, 10, 0.5, 0, 0.5, false),
            new LimitSwitchSensorIORoboRio(9),
            new double[] {0, 1, 3, 9},
            new double[] {0, 0, 0, 0});
    intakeWheels =
        new IntakeSubsystem(new IntakeWheelIOSparkMax(24, 1, 40), new CoralSensorIOSim());

    // Register named commands

    NamedCommands.registerCommand("Extender to L1", ExtenderCommands.setToPoint(extender, "L1"));
    NamedCommands.registerCommand("Extender to L2", ExtenderCommands.setToPoint(extender, "L2"));
    NamedCommands.registerCommand("Extender to L3", ExtenderCommands.setToPoint(extender, "L3"));
    NamedCommands.registerCommand("Extender to L4", ExtenderCommands.setToPoint(extender, "L4"));
    NamedCommands.registerCommand(
        "Extender to Intake", ExtenderCommands.setToPoint(extender, "Intake"));
    NamedCommands.registerCommand(
        "Extender to Process", ExtenderCommands.setToPoint(extender, "Processor"));
    NamedCommands.registerCommand(
        "Extender to L2 Dealgify", ExtenderCommands.setToPoint(extender, "L2 Dealgify"));
    NamedCommands.registerCommand(
        "Extender to L3 Dealgify", ExtenderCommands.setToPoint(extender, "L3 Dealgify"));

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



    var runWheelsCommand = Commands.runEnd(
                () -> {
                  extender.setElevatorVelocity(opCon.getLeftY() * -0.5);
                  extender.setPivotVelocity(opCon.getRightY() * -0.5);
                },
                () -> {
                  extender.setElevatorVelocity(0);
                  extender.setPivotVelocity(0);
                });



    // --- Driver Controls ---

    var hid = driveCon.getHID();
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> hid.getLeftY(),
            () -> hid.getLeftX(),
            () -> -hid.getRightX() * 4,
            () -> driveCon.leftBumper().getAsBoolean(),
            () -> driveCon.rightBumper().getAsBoolean(),
            new double[] {0.1, 0.2, 0.5, 1}));

    // When the b button is pressed, score coral

    driveCon.b().whileTrue(runWheelsCommand);


    // --- Operator Controls ---

    // When the right button is pressed, score coral
    opCon.rightBumper().whileTrue(Commands.print("Score coral"));

    // When the right trigger is held, run intake (spin feeder wheel and EE wheels, will stop when
    // coral is detected in the right place)
    opCon
        .rightTrigger(0.05)
        .whileTrue(
            Commands.runEnd(
                () -> {
                  endEffectorWheels.setVelocity(opCon.getRightTriggerAxis() * 0.2);
                  intakeWheels.setVelocity(opCon.getRightTriggerAxis() * 0.2);
                },
                () -> {
                  endEffectorWheels.setVelocity(0);
                  intakeWheels.setVelocity(0);
                }));


    opCon
        .leftTrigger(0.05)
        .whileTrue(runWheelsCommand);   


                // When the A button is pressed, go to L1
    opCon.a().onTrue(Commands.runOnce(() -> extender.setSetpoint(0)));

    // When the B button is pressed, go to L2
    opCon.b().onTrue(Commands.runOnce(() -> extender.setSetpoint(1)));

    // When the X button is pressed, go to L3
    opCon.x().onTrue(Commands.runOnce(() -> extender.setSetpoint(2)));

    // When the Y button is pressed, go to L4
    opCon.y().onTrue(Commands.runOnce(() -> extender.setSetpoint(3)));

    // Adjust the end effector setpoint with the right and left D-Pad
    opCon.povLeft().onTrue(Commands.runOnce(() -> extender.modifySetpointAngle(-1)));
    opCon.povRight().onTrue(Commands.runOnce(() -> extender.modifySetpointAngle(1)));

    // Adjust the elevator setpoint with the up and down D-Pad
    opCon.povDown().onTrue(Commands.runOnce(() -> extender.modifySetpointHeight(-0.05)));
    opCon.povUp().onTrue(Commands.runOnce(() -> extender.modifySetpointHeight(0.05)));
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
