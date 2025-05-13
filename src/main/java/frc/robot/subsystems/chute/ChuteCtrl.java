package frc.robot.subsystems.chute;

public interface ChuteCtrl {
  public void periodic();

  // System
  public default void extend() {}

  public default void retract() {}

  // Manual
  public default void setVoltage(double volts) {}
}
