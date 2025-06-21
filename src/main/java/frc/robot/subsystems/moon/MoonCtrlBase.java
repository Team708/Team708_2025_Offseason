package frc.robot.subsystems.moon;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class MoonCtrlBase {
  protected final MoonIOInputsAutoLogged inputs;
  protected final MoonIO io;

  public MoonCtrlBase() {
    this.io = Constants.currentMode == Mode.REAL ? new MoonIOReal() : new MoonIOSim();
    inputs = new MoonIOInputsAutoLogged();
    init();
  }

  public MoonCtrlBase(MoonIO io) {
    this.io = io;
    inputs = new MoonIOInputsAutoLogged();
    init();
  }

  protected void init() {}
  ;

  public abstract void periodic();
}
