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
  public static final double maxSpeedMetersPerSec = 4.5;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(18.875);
  public static final double wheelBase = Units.inchesToMeters(24.75);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int pigeonCanId = 4;

  public static final int frontLeftDriveCanId = 12;
  public static final int backLeftDriveCanId = 13;
  public static final int frontRightDriveCanId = 14;
  public static final int backRightDriveCanId = 11;

  public static final int frontLeftTurnCanId = 16;
  public static final int backLeftTurnCanId = 17;
  public static final int frontRightTurnCanId = 18;
  public static final int backRightTurnCanId = 15;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 80;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction = 5.9;
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.8;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.1;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 18.75;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.5;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = Units.lbsToKilograms(138.0);
  public static final double robotMOI = 5.47;
  public static final double wheelCOF = 0.395;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
  public static final double startupPeriodicDelay = 2.0;

  // Pose maps
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

    poseMapRed.put(1, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI)));
    poseMapRed.put(2, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI)));
    poseMapRed.put(3, new Pose2d(0.0, 0.0, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(4, new Pose2d(0.0, 0.0, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(5, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(6, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(7, new Pose2d(0.0, 0.0, new Rotation2d(0)));
    poseMapRed.put(8, new Pose2d(0.0, 0.0, new Rotation2d(0)));
    poseMapRed.put(9, new Pose2d(0.0, 0.0, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(10, new Pose2d(0.0, 0.0, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(11, new Pose2d(0.0, 0.0, new Rotation2d((2 * Math.PI) / 3)));
    poseMapRed.put(12, new Pose2d(0.0, 0.0, new Rotation2d((2 * Math.PI) / 3)));
  }
}
