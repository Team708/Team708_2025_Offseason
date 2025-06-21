package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public class Climber extends SubsystemBase {
  private LoggedTunableBoolean manualMode = new LoggedTunableBoolean("Climber/ManualMode", false);
  private ClimberCtrl climberCtrl;

  public Climber(ClimberCtrl climberCtrl) {
    this.climberCtrl = climberCtrl;
  }

  @Override
  public void periodic() {
    if (manualMode.get() && climberCtrl instanceof ClimberCtrlSystem) {
      climberCtrl = new ClimberCtrlManual();
    } else if (climberCtrl instanceof ClimberCtrlManual) {
      climberCtrl = new ClimberCtrlSystem();
    }
    climberCtrl.periodic();
  }

  public ClimberCtrl getClimberCtrl() {
    return this.climberCtrl;
  }
}
