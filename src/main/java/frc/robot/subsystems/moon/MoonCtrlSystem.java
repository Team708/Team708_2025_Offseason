package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;
import org.littletonrobotics.junction.Logger;

public class MoonCtrlSystem extends SubsystemBase implements MoonCtrl {
  private final MoonIO io;
  private final MoonIOInputsAutoLogged inputs;
  private boolean isCoralMode;
  private double targetRadians;

  public MoonCtrlSystem(MoonIO io) {
    this.io = io;
    inputs = new MoonIOInputsAutoLogged();
    targetRadians = 0.0;
    isCoralMode = true;
    Logger.recordOutput("Moon/TargetPosition", "Unknown");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Moon", inputs);
    Logger.recordOutput("Moon/IsAtTarget", atTargetPosition());

    if (!inputs.reverseLimitReached && targetRadians <= 0) {
      targetRadians -= 1;
    }

    inputs.targetRadians = targetRadians;
  }

  @Override
  public void setTargetPosition(MoonTarget target) {
    Logger.recordOutput("Moon/TargetPosition", target.name());
    targetRadians = target.radians;
  }

  @Override
  public boolean atTargetPosition() {
    return Math.abs(inputs.positionRadians - targetRadians) <= DEADBAND;
  }

  @Override
  public boolean getIsCoralMode() {
    return isCoralMode;
  }

  @Override
  public void setIsCoralMode(boolean isCoralMode) {
    this.isCoralMode = isCoralMode;
  }
}
