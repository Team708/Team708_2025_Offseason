package frc.robot.subsystems.chute;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ChuteCtrlManual extends SubsystemBase implements ChuteCtrl {
  private final ChuteIO io;
  private final ChuteIOInputsAutoLogged inputs;

  public ChuteCtrlManual(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
  }

  @Override
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
