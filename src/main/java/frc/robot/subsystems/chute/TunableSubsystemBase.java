package frc.robot.subsystems.chute;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableBoolean;

public abstract class TunableSubsystemBase<TSystem extends ChuteCtrlSystem,TManual extends ChuteCtrlManual> extends SubsystemBase {
  Class<TSystem> typeTSystem;
  Class<TManual> typeTManual;
  ChuteCtrlBase ctrl;
  boolean isManualMode = false;

  LoggedTunableBoolean manualMode = new LoggedTunableBoolean(this.getName() + " ManualMode", false);

  @Override
  public void periodic() {
    boolean mode = manualMode.get();
    periodicTunable(mode);

    if (mode) {
        if (!isManualMode) {
          ctrl = typeTManual.getConstructor().newInstance();
        }
      } else {
        if (isManualMode) {
          ctrl = typeTSystem.getConstructor().newInstance();
        }
      }
      ctrl.periodic();
  }

  protected abstract void periodicTunable(boolean manualMode);
}
