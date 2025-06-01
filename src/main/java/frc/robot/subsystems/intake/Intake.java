package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeCtrlSystem intakeCtrl;

  public Intake(IntakeCtrlSystem intakeCtrl) {
    this.intakeCtrl = intakeCtrl;
  }

  @Override
  public void periodic() {
    intakeCtrl.periodic();
  }

  public IntakeCtrlSystem getIntakeCtrl() {
    return this.intakeCtrl;
  }
}
