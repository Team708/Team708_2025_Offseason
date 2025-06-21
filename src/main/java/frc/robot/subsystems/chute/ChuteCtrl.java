package frc.robot.subsystems.chute;

public interface IChuteCtrl {
  public void periodic();

  public default void extend() {}

  public default void retract() {}

  public default void setVoltage(double volts) {}

  public default double getPosition() {
    return 0.0;
  }

  public default boolean isExtended() {
    return false;
  }

  public default boolean isRetracted() {
    return false;
  }
}
