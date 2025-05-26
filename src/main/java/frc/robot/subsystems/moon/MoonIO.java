package frc.robot.subsystems.moon;

import org.littletonrobotics.junction.AutoLog;

public interface MoonIO {
  @AutoLog
  public static class MoonIOInputs {}

  public default void updateInputs(MoonIOInputs inputs) {}
}
