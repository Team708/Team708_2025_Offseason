package frc.robot.subsystems.template;

import org.littletonrobotics.junction.AutoLog;

public interface TemplateIO {
  @AutoLog
  public static class TemplateIOInputs {}

  public default void updateInputs(TemplateIOInputs inputs) {}
}
