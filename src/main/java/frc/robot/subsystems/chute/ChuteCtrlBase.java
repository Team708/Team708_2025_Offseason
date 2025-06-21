package frc.robot.subsystems.chute;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class ChuteCtrlBase {
  protected final ChuteIOInputsAutoLogged inputs;
  protected final ChuteIO io;

  public ChuteCtrlBase() {
    this.io = Constants.currentMode == Mode.REAL ? new ChuteIOReal() : new ChuteIOSim();
    inputs = new ChuteIOInputsAutoLogged();
    init();
  }

  public ChuteCtrlBase(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
    init();
  }

  protected void init() {}
  ;

  public abstract void periodic();
}
