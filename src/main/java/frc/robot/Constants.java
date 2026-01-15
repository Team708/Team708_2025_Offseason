// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
  public static final boolean TUNING_MODE = true;
  public static final boolean DISABLE_HAL = false;
  public static final boolean CHUTE_MANUAL_MODE = false;
  public static final boolean ELEVATOR_MANUAL_MODE = false;
  public static final boolean MOON_MANUAL_MODE = false;
  public static final boolean CLIMBER_MANUAL_MODE = false;
  public static final DCMotor K1_VORTEX = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 1);
  public static final DCMotor K2_VORTEX = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 2);
  public static final double BACKGROUND_THREAD_PERIOD = 2;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
