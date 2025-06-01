package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double rpm = 0.0;
    public double positionRad = 0.0;
    public boolean beamTriggered = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
