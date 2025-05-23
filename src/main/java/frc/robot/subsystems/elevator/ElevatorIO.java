package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motor1Connected = false;
    public boolean motor2Connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double rpm = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
  public default void setVoltage(double volts) {}
}
