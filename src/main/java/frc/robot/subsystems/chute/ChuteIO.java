package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
  @AutoLog
  public static class ChuteIOInputs {
    public boolean connected = false;
    public boolean isFullyExtended = false;
    public boolean isFullyRetracted = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionInches = 0.0;
    public double velocityInchesPerSecond = 0.0;
    public double rpm = 0.0;
  }

  public default void updateInputs(ChuteIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
