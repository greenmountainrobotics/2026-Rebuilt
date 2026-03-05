package frc.robot.subsystems.launch;

public package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.angleModulus;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX topSpinMotor =
      new TalonFX(ShooterConstants.TopMotorCanID);
  private final TalonFX bottomSpinMotor =
      new TalonFX(ShooterConstants.BottomMotorCanID);

  /*  private final CANSparkMax articulationMotor =
  new CANSparkMax(
      IdConstants.CANId.ShooterArticulationMotorId, CANSparkLowLevel.MotorType.kBrushed);*/

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final DutyCycleEncoder articulationEncoder;

  public ShooterIOReal() {
    topEncoder = topSpinMotor.getEncoder();
    bottomEncoder = bottomSpinMotor.getEncoder();
    topSpinMotor.setInverted(true);
    articulationEncoder = new DutyCycleEncoder(IdConstants.DIOId.ShooterArticulationEncoderId);

    topEncoder.setPositionConversionFactor(1.0 / 42);
    bottomEncoder.setPositionConversionFactor(1.0 / 42);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.bottomPositionRad =
        Units.rotationsToRadians(bottomEncoder.getPosition()) / ShooterConstants.ShooterGearRatio;
    inputs.topPositionRad =
        Units.rotationsToRadians(topEncoder.getPosition()) / ShooterConstants.ShooterGearRatio;
    inputs.articulationPosition =
        Rotation2d.fromRadians(
            angleModulus(
                Rotation2d.fromRotations(
                        -articulationEncoder.getAbsolutePosition()
                            + ShooterConstants.AbsoluteEncoderOffset.getRotations())
                    .getRadians()));

    inputs.bottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity())
            / ShooterConstants.ShooterGearRatio;
    inputs.topVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity())
            / ShooterConstants.ShooterGearRatio;

    inputs.bottomAppliedVolts =
        bottomSpinMotor.getAppliedOutput() * bottomSpinMotor.getBusVoltage();
    inputs.topAppliedVolts = topSpinMotor.getAppliedOutput() * topSpinMotor.getBusVoltage();
  }

  @Override
  public void setTopVoltage(double volts) {
    topSpinMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomSpinMotor.setVoltage(volts);
  }

  @Override
  public void setArticulationVoltage(double volts) {
    /*articulationMotor.setVoltage(volts);*/
  }
} {
    
}
