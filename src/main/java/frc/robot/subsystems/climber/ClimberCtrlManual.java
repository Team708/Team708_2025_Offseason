package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberCtrlManual extends SubsystemBase implements ClimberCtrl {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public ClimberCtrlManual(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
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

  @Override
  public void setServo(boolean isUnlocked) {
    io.setServo(isUnlocked);
  }
}
