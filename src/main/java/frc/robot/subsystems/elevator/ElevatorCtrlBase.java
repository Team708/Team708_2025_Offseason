package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class ElevatorCtrlBase {
  protected final ElevatorIOInputsAutoLogged inputs;
  protected final ElevatorIO io;

  public ElevatorCtrlBase() {
    this.io = Constants.currentMode == Mode.REAL ? new ElevatorIOReal() : new ElevatorIOSim();
    inputs = new ElevatorIOInputsAutoLogged();
    init();
  }

  public ElevatorCtrlBase(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    init();
  }

  protected void init() {}
  ;

  public abstract void periodic();
}
