// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/** Intake IO implementation for one NEO on SparkMax (voltage control). */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;
  private static final double RAD_PER_SEC_PER_RPM = RADIANS_PER_ROTATION / 60.0;

  private final SparkMax motor = new SparkMax(intakeMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  public IntakeIOSparkMax() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config.inverted(false);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(motor, encoder::getPosition, (v) -> inputs.positionRad = v * RADIANS_PER_ROTATION);
    ifOk(motor, encoder::getVelocity, (v) -> inputs.velocityRadPerSec = v * RAD_PER_SEC_PER_RPM);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    inputs.currentAmps = new double[1];
    ifOk(motor, motor::getOutputCurrent, (v) -> inputs.currentAmps[0] = v);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
