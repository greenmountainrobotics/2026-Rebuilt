package frc.robot.subsystems.launch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  // TODO: set constants
  private final DCMotorSim topSpinMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0025);
  private final DCMotorSim bottomSpinMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0025);
  private final DCMotorSim articulationMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private double articulationAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSpinMotor.update(LOOP_PERIOD_SECS);
    bottomSpinMotor.update(LOOP_PERIOD_SECS);
    articulationMotor.update(LOOP_PERIOD_SECS);

    inputs.articulationPosition = new Rotation2d(articulationMotor.getAngularPositionRad());
    inputs.articulationAppliedVolts = articulationAppliedVolts;
    inputs.articulationCurrentAmps =
        new double[] {Math.abs(articulationMotor.getCurrentDrawAmps())};

    inputs.topPositionRad = topSpinMotor.getAngularPositionRad();
    inputs.topVelocityRadPerSec = topSpinMotor.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = new double[] {Math.abs(topSpinMotor.getCurrentDrawAmps())};

    inputs.bottomPositionRad = bottomSpinMotor.getAngularPositionRad();
    inputs.bottomVelocityRadPerSec = bottomSpinMotor.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = new double[] {Math.abs(bottomSpinMotor.getCurrentDrawAmps())};
  }

  @Override
  public void setArticulationVoltage(double volts) {
    articulationAppliedVolts = volts;
    articulationMotor.setInputVoltage(volts);
  }

  @Override
  public void setTopVoltage(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12, 12);
    topSpinMotor.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12, 12);
    bottomSpinMotor.setInputVoltage(bottomAppliedVolts);
  }
}