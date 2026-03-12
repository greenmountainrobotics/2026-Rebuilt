// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feed;

import static frc.robot.subsystems.feed.FeedConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Feed IO implementation for two Kraken (Talon FX) motors. */
public class FeedIOTalonFX implements FeedIO {
  private final TalonFX leader = new TalonFX(feedLeaderCanId);
  private final TalonFX follower = new TalonFX(feedFollowerCanId);

  private final StatusSignal<Angle> position = leader.getPosition();
  private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public FeedIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    config.MotorOutput.Inverted =
        followerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeedIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, leaderCurrent, followerCurrent);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {
          leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble(),
        };
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }
}
