package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class ClimberCtrlBase {
  protected final ClimberIOInputsAutoLogged inputs;
  protected final ClimberIO io;

  public ClimberCtrlBase() {
    this.io = Constants.currentMode == Mode.REAL ? new ClimberIOReal() : new ClimberIOSim();
    inputs = new ClimberIOInputsAutoLogged();
    init();
  }

  public ClimberCtrlBase(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
    init();
  }

  protected void init() {}
  ;

  public abstract void periodic();
}
