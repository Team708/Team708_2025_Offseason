package frc.robot.subsystems.chute;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public class Chute extends SubsystemBase {
  LoggedTunableBoolean manualMode = new LoggedTunableBoolean("Chute/ManualMode", false);
  private ChuteCtrl chuteCtrl;

  public Chute(ChuteCtrl chuteCtrl) {
    this.chuteCtrl = chuteCtrl;
  }

  @Override
  public void periodic() {
    if (manualMode.get() && chuteCtrl instanceof ChuteCtrlSystem) {
      chuteCtrl = new ChuteCtrlManual();
    } else if (chuteCtrl instanceof ChuteCtrlManual) {
      chuteCtrl = new ChuteCtrlSystem();
    }

    chuteCtrl.periodic();
  }

  public ChuteCtrl getChuteCtrl() {
    return this.chuteCtrl;
  }
}
