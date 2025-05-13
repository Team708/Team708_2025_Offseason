package frc.robot.subsystems.chute;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  private ChuteCtrl chuteCtrl;

  public Chute(ChuteCtrl chuteCtrl) {
    this.chuteCtrl = chuteCtrl;
  }

  @Override
  public void periodic() {
    chuteCtrl.periodic();
  }

  public ChuteCtrl getChuteCtrl() {
    return this.chuteCtrl;
  }
}
