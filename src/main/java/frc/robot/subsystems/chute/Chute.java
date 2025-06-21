package frc.robot.subsystems.chute;

public class Chute extends TunableSubsystemBase {
  private IChuteCtrl chuteCtrl;

  public Chute(IChuteCtrl chuteCtrl) {
    this.chuteCtrl = chuteCtrl;
  }

  @Override
  public void periodicTunable(boolean manualMode) {
    if (manualMode) {
      if (chuteCtrl instanceof ChuteCtrlSystem) {
        chuteCtrl = new ChuteCtrlManual();
      }
    } else {
      if (chuteCtrl instanceof ChuteCtrlManual) {
        chuteCtrl = new ChuteCtrlSystem();
      }
    }
    chuteCtrl.periodic();
  }

  public IChuteCtrl getChuteCtrl() {
    return this.chuteCtrl;
  }
}
