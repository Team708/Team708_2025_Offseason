package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public class Elevator extends SubsystemBase {
  private LoggedTunableBoolean manualMode = new LoggedTunableBoolean("Elevator/ManualMode", false);
  private ElevatorCtrl elevatorCtrl;

  public Elevator() {
    elevatorCtrl = manualMode.get() ? new ElevatorCtrlManual() : new ElevatorCtrlSystem();
  }

  @Override
  public void periodic() {
    if (manualMode.get() && elevatorCtrl instanceof ElevatorCtrlSystem) {
      elevatorCtrl = new ElevatorCtrlManual();
    } else if (elevatorCtrl instanceof ElevatorCtrlManual) {
      elevatorCtrl = new ElevatorCtrlSystem();
    }
    elevatorCtrl.periodic();
  }

  public ElevatorCtrl getElevatorCtrl() {
    return this.elevatorCtrl;
  }
}
