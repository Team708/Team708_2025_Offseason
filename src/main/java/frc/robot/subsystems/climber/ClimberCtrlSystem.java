package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberCtrlSystem extends SubsystemBase implements ClimberCtrl {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public ClimberCtrlSystem(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
