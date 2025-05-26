package frc.robot.subsystems.chute;

public interface ChuteCtrl {
  public void periodic();

  public default void extend() {}

  public default void retract() {}

  public default void setVoltage(double volts) {}
}
