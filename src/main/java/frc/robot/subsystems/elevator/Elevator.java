package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorCtrl elevatorCtrl;

  public Elevator(ElevatorCtrl elevatorCtrl) {
    this.elevatorCtrl = elevatorCtrl;
  }

  @Override
  public void periodic() {
    elevatorCtrl.periodic();
  }

  public ElevatorCtrl getElevatorCtrl() {
    return this.elevatorCtrl;
  }
}
