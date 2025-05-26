package frc.robot.subsystems.moon;

import org.littletonrobotics.junction.AutoLog;

public interface MoonIO {
  @AutoLog
  public static class MoonIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double rpm = 0.0;
  }

  public default void updateInputs(MoonIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
  ;
}
