package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeCtrl intakeCtrl;

  public Intake(IntakeCtrl intakeCtrl) {
    this.intakeCtrl = intakeCtrl;
  }

  @Override
  public void periodic() {
    intakeCtrl.periodic();
  }

  public IntakeCtrl getIntakeCtrl() {
    return this.intakeCtrl;
  }
}
