// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

/** Simulated intake IO. Uses a simple first-order model for velocity/position. */
public class IntakeIOSim implements IntakeIO {
  private static final double MAX_VELOCITY_RAD_PER_SEC = 100.0;
  private static final double DT = 0.02;

  private double appliedVolts = 0.0;
  private double positionRad = 0.0;
  private double velocityRadPerSec = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    velocityRadPerSec =
        MathUtil.clamp(
            (appliedVolts / 12.0) * MAX_VELOCITY_RAD_PER_SEC,
            -MAX_VELOCITY_RAD_PER_SEC,
            MAX_VELOCITY_RAD_PER_SEC);
    positionRad += velocityRadPerSec * DT;

    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {appliedVolts / 12.0 * 5.0, appliedVolts / 12.0 * 5.0};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
