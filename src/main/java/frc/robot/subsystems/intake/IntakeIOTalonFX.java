// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Intake IO implementation for one Kraken (TalonFX) motor. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor = new TalonFX(intakeMotorCanId);

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 1.0;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, motorCurrent);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {motorCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
