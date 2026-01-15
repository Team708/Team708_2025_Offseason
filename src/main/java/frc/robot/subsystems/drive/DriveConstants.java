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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class DriveConstants {
  public static final double MAX_SPEED_METERS_PER_SEC = 4.5;
  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
  public static final double TRACK_WIDTH = Units.inchesToMeters(18.875);
  public static final double WHEELBASE = Units.inchesToMeters(24.75);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEELBASE / 2.0);
  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH / 2.0, WHEELBASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEELBASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEELBASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEELBASE / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d FRONT_LEFT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d FRONT_RIGHT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d BACK_LEFT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d BACK_RIGHT_ZERO_ROTATION = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int PIGEON_CAN_ID = 4;

  public static final int FRONT_LEFT_DRIVE_CAN_ID = 12;
  public static final int BACK_LEFT_DRIVE_CAN_ID = 13;
  public static final int FRONT_RIGHT_DRIVE_CAN_ID = 14;
  public static final int BACK_RIGHT_DRIVE_CAN_ID = 11;

  public static final int FRONT_LEFT_TURN_CAN_ID = 16;
  public static final int BACK_LEFT_TURN_CAN_ID = 17;
  public static final int FRONT_RIGHT_TURN_CAN_ID = 18;
  public static final int BACK_RIGHT_TURN_CAN_ID = 15;

  // Drive motor configuration
  public static final int DRIVE_MOTOR_CURRENT_LIMIT = 80;
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
  public static final double DRIVE_MOTOR_REDUCTION = 5.9;
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double DRIVE_ENCODER_POSITION_FACTOR =
      2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations -> Wheel Radians
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double DRIVE_KP = 0.0;
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KS = 0.0;
  public static final double DRIVE_KV = 0.1;
  public static final double DRIVE_SIM_KP = 0.8;
  public static final double DRIVE_SIM_KD = 0.0;
  public static final double DRIVE_SIM_KS = 0.0;
  public static final double DRIVE_SIM_KV = 0.1;

  // Turn motor configuration
  public static final boolean TURN_INVERTED = true;
  public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
  public static final double TURN_MOTOR_REDUCTION = 18.75;
  public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean TURN_ENCODER_INVERTED = false;
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double TURN_KP = 0.5;
  public static final double TURN_KD = 0.0;
  public static final double TURN_SIM_KP = 8.0;
  public static final double TURN_SIM_KD = 0.0;
  public static final double TURN_PID_MIN_INPUT = 0; // Radians
  public static final double TURN_PID_MAX_INPUT = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double ROBOT_MASS_KG = Units.lbsToKilograms(138.0);
  public static final double ROBOT_MOI_KG_METERS_SQUARED = 5.47;
  public static final double WHEEL_COEFFICENT_OF_FRICTION = 0.395;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI_KG_METERS_SQUARED,
          new ModuleConfig(
              WHEEL_RADIUS_METERS,
              MAX_SPEED_METERS_PER_SEC,
              WHEEL_COEFFICENT_OF_FRICTION,
              DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
              DRIVE_MOTOR_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);
  public static final double STARTUP_PERIODIC_DELAY = 2.0;

  // Poses
  public static final Pose2d STARTING_POSE_LEFT_RED = new Pose2d(10.42, 5.4, new Rotation2d(0));
  public static final Pose2d STARTING_POSE_LEFT_BLUE =
      new Pose2d(7.121, 5.4, new Rotation2d(Math.toRadians(180)));
  public static final Pose2d STARTING_POSE_CENTER_RED = new Pose2d(10.42, 4.0, new Rotation2d(0));
  public static final Pose2d STARTING_POSE_CENTER_BLUE =
      new Pose2d(7.121, 4.0, new Rotation2d(Math.toRadians(180)));
  public static final Pose2d STARTING_POSE_RIGHT_RED = new Pose2d(10.42, 2.6, new Rotation2d(0));
  public static final Pose2d STARTING_POSE_RIGHT_BLUE =
      new Pose2d(7.121, 2.6, new Rotation2d(Math.toRadians(180)));
  public static final Pose2d FEEDER_LEFT_RED =
      new Pose2d(16.147, 0.84, new Rotation2d(Math.toRadians(125.0)));
  ;
  public static final Pose2d FEEDER_RIGHT_RED =
      new Pose2d(16.147, 7.16, new Rotation2d(Math.toRadians(-125.0)));
  public static final Pose2d FEEDER_LEFT_BLUE =
      new Pose2d(1.353, 7.16, new Rotation2d(Math.toRadians(-55.0)));
  public static final Pose2d FEEDER_RIGHT_BLUE =
      new Pose2d(1.353, 0.84, new Rotation2d(Math.toRadians(55.0)));

  public static HashMap<Integer, Pose2d> poseMapBlue = new HashMap<Integer, Pose2d>();
  public static HashMap<Integer, Pose2d> poseMapRed = new HashMap<Integer, Pose2d>();

  static {
    poseMapBlue.put(1, new Pose2d(3.12, 4.20, new Rotation2d(0)));
    poseMapBlue.put(2, new Pose2d(3.12, 3.84, new Rotation2d(0)));
    poseMapBlue.put(3, new Pose2d(3.63, 2.92, new Rotation2d(Math.PI / 3)));
    poseMapBlue.put(4, new Pose2d(3.96, 2.73, new Rotation2d(Math.PI / 3)));
    poseMapBlue.put(5, new Pose2d(5.04, 2.76, new Rotation2d((2 * Math.PI) / 3)));
    poseMapBlue.put(6, new Pose2d(5.32, 2.90, new Rotation2d((2 * Math.PI) / 3)));
    poseMapBlue.put(7, new Pose2d(5.88, 3.84, new Rotation2d(-Math.PI)));
    poseMapBlue.put(8, new Pose2d(5.88, 4.20, new Rotation2d(-Math.PI)));
    poseMapBlue.put(9, new Pose2d(5.32, 5.198, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapBlue.put(10, new Pose2d(5.04, 5.407, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapBlue.put(11, new Pose2d(3.96, 5.407, new Rotation2d(-Math.PI / 3)));
    poseMapBlue.put(12, new Pose2d(3.63, 5.198, new Rotation2d(-Math.PI / 3)));

    poseMapRed.put(1, new Pose2d(14.44, 3.84, new Rotation2d(-Math.PI)));
    poseMapRed.put(2, new Pose2d(14.44, 4.20, new Rotation2d(-Math.PI)));
    poseMapRed.put(3, new Pose2d(13.93, 5.198, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(4, new Pose2d(13.6, 5.407, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(5, new Pose2d(12.52, 5.407, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(6, new Pose2d(12.24, 5.198, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(7, new Pose2d(11.68, 4.20, new Rotation2d(0)));
    poseMapRed.put(8, new Pose2d(11.68, 3.84, new Rotation2d(0)));
    poseMapRed.put(9, new Pose2d(12.24, 2.92, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(10, new Pose2d(12.52, 2.73, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(11, new Pose2d(13.6, 2.76, new Rotation2d((2 * Math.PI) / 3)));
    poseMapRed.put(12, new Pose2d(13.93, 2.90, new Rotation2d((2 * Math.PI) / 3)));
  }
}
