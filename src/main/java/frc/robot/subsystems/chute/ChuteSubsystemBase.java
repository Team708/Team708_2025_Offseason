package frc.robot.subsystems.chute;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public abstract class ChuteSubsystemBase extends SubsystemBase {
  ChuteCtrlBase ctrl;
  LoggedTunableBoolean manualMode = new LoggedTunableBoolean(this.getName() + " ManualMode", false);

  @Override
  public void periodic() {
    boolean isManualMode = manualMode.get();
    periodicTunable(isManualMode);

    if (isManualMode && ctrl instanceof ChuteCtrlSystem) {
      ctrl = new ChuteCtrlManual();
    } else if (!isManualMode && ctrl instanceof ChuteCtrlManual) {
      ctrl = new ChuteCtrlSystem();
    }

    ctrl.periodic();
  }

  protected abstract void periodicTunable(boolean manualMode);
}
