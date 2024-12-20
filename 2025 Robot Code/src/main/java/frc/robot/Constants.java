// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.drive.VisionIO;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum Modes {
    SIM,
    REAL
  }

  public static final class CANBusIDs {
    public static final int kPigeon2CanID = 14;

  }

  public static final class PhysicalRobotCharacteristics {
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.5);
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.5);

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0),
            new Translation2d(kTrackWidthMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0),
            new Translation2d(-kTrackWidthMeters / 2.0, -kWheelBaseMeters / 2.0)
        };

    public static final double kDriveBaseRadiusMeters = Math.hypot(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0);

    public static final double kMaxLinearSpeedMetersPerSec = Units.feetToMeters(15);

    public static final double kMaxAngularSpeedRadPerSec = kMaxLinearSpeedMetersPerSec / kDriveBaseRadiusMeters;
  }
  public static final class KeyPoints {
    //index of name matches with index of Pose2d. coordinates are not final, nor the rotation value
    public static final String[] keyPointsName = {"homeSpeaker", "homeAmp", "homeSource", "homeStage"};
    public static final Pose2d[] positions = new Pose2d[] {
      new Pose2d(1.0, 1.0, new Rotation2d(0.0)),
      new Pose2d(2.0, 2.0, new Rotation2d(0.0)),
      new Pose2d(3.0, 3.0, new Rotation2d(0.0)),
      new Pose2d(4.0, 4.0, new Rotation2d(0.0))
    };
    //length of field is 16.592 meters
    
  }

  public static class DriveConstants {
                // Both sets of gains need to be tuned to your individual robot.

                // The steer motor uses any SwerveModule.SteerRequestType control request with the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0.2)
                        .withKS(0).withKV(2).withKA(0);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)
                        .withKS(0).withKV(0).withKA(0);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final double kSlipCurrentA = 40.0;
                public static final double kSupplyCurrentA = 40.0;

                // Theoretical free speed (m/s) at 12v applied output;
                // This needs to be tuned to your individual robot
                public static final double kSpeedAt12VoltsMps = PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec;
                public static final double kMaxAngularSpeedRadPerSec = PhysicalRobotCharacteristics.kMaxAngularSpeedRadPerSec;

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio =  3.5714285714285716;

                private static final double kDriveGearRatio = 6.746031746031747;
                private static final double kSteerGearRatio = 21.428571428571427;
                private static final double kWheelRadiusInches = 2;

                private static final boolean kSteerMotorReversed = true;
                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                private static final String kCANbusName = "Drive";
                private static final int kPigeonId = 14;


                // These are only used for simulation
                private static final double kSteerInertia = 0.00001;
                private static final double kDriveInertia = 0.001;
                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.25;
                private static final double kDriveFrictionVoltage = 0.25;

                private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                        .withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

                private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

                


                // Front Left
                private static final int kFrontLeftDriveMotorId = 1;
                private static final int kFrontLeftSteerMotorId = 2;
                private static final int kFrontLeftEncoderId = 3;
                private static final double kFrontLeftEncoderOffset = 0.2021484375;

                private static final double kFrontLeftXPosInches = 11.25;
                private static final double kFrontLeftYPosInches = 11.25;

                // Front Right
                private static final int kFrontRightDriveMotorId = 4;
                private static final int kFrontRightSteerMotorId = 5;
                private static final int kFrontRightEncoderId = 6;
                private static final double kFrontRightEncoderOffset = 0.197509765625;

                private static final double kFrontRightXPosInches = 11.25;
                private static final double kFrontRightYPosInches = -11.25;

                // Back Left
                private static final int kBackLeftDriveMotorId = 7;
                private static final int kBackLeftSteerMotorId = 8;
                private static final int kBackLeftEncoderId = 9;
                private static final double kBackLeftEncoderOffset = -0.215576171875;

                private static final double kBackLeftXPosInches = -11.25;
                private static final double kBackLeftYPosInches = 11.25;

                // Back Right
                private static final int kBackRightDriveMotorId = 10;
                private static final int kBackRightSteerMotorId = 11;
                private static final int kBackRightEncoderId = 12;
                private static final double kBackRightEncoderOffset = 0.269287109375;

                private static final double kBackRightXPosInches = -11.25;
                private static final double kBackRightYPosInches = -11.25;


                private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
                private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
                private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
                private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

                public static final SwerveDriveSubsystem DriveTrain = new SwerveDriveSubsystem(DrivetrainConstants, new VisionIO(), FrontLeft,
                        FrontRight, BackLeft, BackRight);

                
        }


        public static class Vision {
                public static final String kCameraName = "VisionCam";
                // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
                // TODO: Configure this
                public static final Transform3d kRobotToCam =
                        new Transform3d(new Translation3d(Units.inchesToMeters(-4.25), Units.inchesToMeters(-12.125), Units.inchesToMeters(12.25)), new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));

                // The layout of the AprilTags on the field
                public static final AprilTagFieldLayout kTagLayout =
                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

                // The standard deviations of our vision estimated poses, which affect correction rate
                // (Fake values. Experiment and determine estimation noise on an actual robot.)
                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }


    
}
