package frc.robot.subsystems.moon;

import org.littletonrobotics.junction.AutoLog;

public interface MoonIO {
  @AutoLog
  public static class MoonIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionRadians = 0.0;
    public double positionDegrees = 0.0;
    public double rpm = 0.0;
    public boolean reverseLimitReached = false;
    public boolean forwardLimitReached = false;
    public double targetRadians = 0.0;
  }

  public default void updateInputs(MoonIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setTarget(double radians) {}
}
