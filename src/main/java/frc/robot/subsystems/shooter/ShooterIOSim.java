// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

/** Simulated shooter IO with velocity setpoint and simple first-order response. */
public class ShooterIOSim implements ShooterIO {
  private static final double DT = 0.02;
  private static final double TIME_CONSTANT = 0.05;

  private double velocitySetpointRadPerSec = 0.0;
  private double velocityRadPerSec = 0.0;
  private double positionRad = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    velocityRadPerSec +=
        (velocitySetpointRadPerSec - velocityRadPerSec) * Math.min(1.0, DT / TIME_CONSTANT);
    positionRad += velocityRadPerSec * DT;
    appliedVolts = MathUtil.clamp(velocityRadPerSec * 0.012 + 0.05, -12.0, 12.0);

    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps =
        new double[] {
          appliedVolts / 12.0 * 5.0, appliedVolts / 12.0 * 5.0, appliedVolts / 12.0 * 5.0
        };
  }

  @Override
  public void setVelocity(double radPerSec, double ffVolts) {
    velocitySetpointRadPerSec = radPerSec;
  }
}
