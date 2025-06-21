package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

public class ClimberCtrlManual extends ClimberCtrlBase implements ClimberCtrl {

  @Override
  protected void init() {
    io.setServo(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  @Override
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
