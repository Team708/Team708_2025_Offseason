package frc.robot.subsystems.climber;

public interface ClimberCtrl {
  public void periodic();
  public default void extend() {}
  public default void retract() {}
}
