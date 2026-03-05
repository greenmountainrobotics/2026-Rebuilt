
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Runs both intake motors open-loop at the given voltage (e.g. -12 to 12). */
  public void runVoltage(double volts) {
    io.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  /** Stops the intake (0 V). */
  public void stop() {
    runVoltage(0.0);
  }

  /**
   * Returns a command that runs the intake at a voltage supplied each period (e.g. from trigger
   * axis). The command never finishes; cancel by interruption or by binding with .whileTrue().
   * Stops the intake when the command ends.
   */
  public Command runWithVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> runVoltage(voltageSupplier.getAsDouble())).finallyDo(() -> stop());
  }
}
