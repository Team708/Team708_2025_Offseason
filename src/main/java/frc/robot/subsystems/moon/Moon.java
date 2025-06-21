package frc.robot.subsystems.moon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public class Moon extends SubsystemBase {
  private LoggedTunableBoolean manualMode = new LoggedTunableBoolean("Moon/ManualMode", false);
  private MoonCtrl moonCtrl;

  public Moon(MoonCtrl moonCtrl) {
    this.moonCtrl = moonCtrl;
  }

  @Override
  public void periodic() {
    if (manualMode.get() && moonCtrl instanceof MoonCtrlSystem) {
      moonCtrl = new MoonCtrlManual();
    } else if (moonCtrl instanceof MoonCtrlManual) {
      moonCtrl = new MoonCtrlSystem();
    }
    moonCtrl.periodic();
  }

  public MoonCtrl getMoonCtrl() {
    return this.moonCtrl;
  }
}
