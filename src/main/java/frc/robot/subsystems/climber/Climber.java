package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberCtrl climberCtrl;

  public Climber(ClimberCtrl climberCtrl) {
    this.climberCtrl = climberCtrl;
  }

  @Override
  public void periodic() {}

  public ClimberCtrl getChuteCtrl() {
    return this.climberCtrl;
  }
}
