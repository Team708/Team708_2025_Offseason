package frc.robot.subsystems.moon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MoonCtrlManual extends SubsystemBase implements MoonCtrl {
  private final MoonIO io;
  private final MoonIOInputsAutoLogged inputs;

  public MoonCtrlManual(MoonIO io) {
    this.io = io;
    inputs = new MoonIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Moon", inputs);
  }
}
