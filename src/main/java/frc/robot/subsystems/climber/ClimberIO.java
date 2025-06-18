package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionRadians = 0.0;
    public double positionDegrees = 0.0;
    public double rpm = 0.0;
    public boolean reverseLimitReached = false;
    public boolean forwardLimitReached = false;
    public boolean servoUnlocked = false;
    public boolean beamBreak1 = true;
    public boolean beamBreak2 = true;
    public boolean cageLimit1 = true;
    public boolean cageLimit2 = true;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setServo(boolean isUnlocked) {}
}
