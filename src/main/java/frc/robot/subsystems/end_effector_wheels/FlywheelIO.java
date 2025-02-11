// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.end_effector_wheels;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public Rotation2d relativeEncoderPosition = new Rotation2d();
    public Rotation2d absoluteEncoderPosition = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void runVelocity(double velocityRadPerSec) {}

  default void runPosition(Rotation2d position) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}

  default void stop() {}
}
