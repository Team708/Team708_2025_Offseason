package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class IntakeCtrlBase {
  protected final IntakeIOInputsAutoLogged inputs;
  protected final IntakeIO io;

  public IntakeCtrlBase() {
    this.io = Constants.currentMode == Mode.REAL ? new IntakeIOReal() : new IntakeIOSim();
    inputs = new IntakeIOInputsAutoLogged();
    init();
  }

  public IntakeCtrlBase(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    init();
  }

  protected void init() {}
  ;

  public abstract void periodic();
}
