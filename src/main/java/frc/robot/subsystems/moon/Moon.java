package frc.robot.subsystems.moon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Moon extends SubsystemBase {
  private MoonCtrl moonCtrl;

  public Moon(MoonCtrl moonCtrl) {
    this.moonCtrl = moonCtrl;
  }

  @Override
  public void periodic() {
    moonCtrl.periodic();
  }

  public MoonCtrl getMoonCtrl() {
    return this.moonCtrl;
  }
}
