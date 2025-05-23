package frc.robot.subsystems.elevator;

public interface ElevatorCtrl {
  public void periodic();

  public default void changeElevatorPosition(double meters) {}
  ;
}
