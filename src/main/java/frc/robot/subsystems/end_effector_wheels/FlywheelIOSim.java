package frc.robot.subsystems.end_effector_wheels;
// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  private static final PIDController velocityController = new PIDController(0.1, 0.0, 0.0);

  private double setpointRadPerSec = 0.0;

  public FlywheelIOSim(DCMotor motorModel, double reduction, double moi) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setpointRadPerSec = 0.0;
    }

    inputs.connected = true;
    sim.update(250);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();

    sim.setInputVoltage(
        velocityController.calculate(sim.getAngularVelocityRadPerSec(), setpointRadPerSec));
  }

  @Override
  public void runVelocity(double velocity) {
    setpointRadPerSec = velocity;
  }
}
