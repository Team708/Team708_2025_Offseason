package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

public class ChuteCtrlManual extends ChuteCtrlBase implements IChuteCtrl {
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
