// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Modes;
import frc.robot.commands.JoystickDriveCmd;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Modes mode;
  
  
  // private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  // private double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);


  private final SwerveDriveSubsystem drivetrain = DriveConstants.DriveTrain; // My drivetrain


  //AutoCreator autoCreator;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Utils.isSimulation()) {
      mode = Modes.SIM;
    } else {
      mode = Modes.REAL;
    }

    if (mode == Modes.REAL){
      
    }
    else{

    }

    // Constants.DriveConstants.putValues();



    // Configure the trigger bindings
    configureBindings();


    //autoCreator = new AutoCreator(shooter, drivetrain, climber);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    var operatorHID = operatorController.getHID();
    var driverHID = driverController.getHID();
    
    drivetrain.setDefaultCommand(new JoystickDriveCmd(drivetrain, driverHID::getLeftY, driverHID::getLeftX, 
        driverHID::getRightX, driverHID::getRightBumper, driverHID::getLeftBumper, 
        () -> !(driverHID.getLeftTriggerAxis() > 0.5), () -> (driverHID.getPOV() == 270)));
    
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    

  //   //return autoCreator.createAuto();
  // }

    
}}