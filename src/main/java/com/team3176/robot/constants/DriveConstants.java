// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team3176.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.team3176.robot.subsystems.drivetrain.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
public class DriveConstants {
  public int SERIAL;
  public int THRUST_CID;
  public String THRUST_CBN;
  public int AZIMUTH_CID;
  public String AZIMUTH_CBN;
  public int CANCODER_CID;
  public String CANCODER_CBN;
  public double OFFSET;
//  public double ROBOT_MASS_KG;
//  public double ROBOT_MOI;
//  public double WHEEL_COF;
/*
 "robotWidth": 0.9,
  "robotLength": 0.9,
  "holonomicMode": true,
  "pathFolders": [],
  "autoFolders": [],
  "defaultMaxVel": 3.0,
  "defaultMaxAccel": 3.0,
  "defaultMaxAngVel": 540.0,
  "defaultMaxAngAccel": 720.0,
  "robotMass": 74.088,
  "robotMOI": 6.883,
  "robotWheelbase": 0.546,
  "robotTrackwidth": 0.546,
  "driveWheelRadius": 0.048,
  "driveGearing": 5.143,
  "maxDriveSpeed": 5.45,
  "driveMotorType": "krakenX60FOC",
  "driveCurrentLimit": 60.0,
  "wheelCOF": 1.2
*/

  public static double SWERVEPOD_AZIMUTH_REDUCTION = 12.8;
  //public static double SWERVEPOD_AZIMUTH_REDUCTION = (150.0 / 7.0);
  public static double SWERVEPOD_AZIMUTH_CURRENTLIMIT = 80;
  public static boolean SWERVEPOD_AZIMUTH_INVERTED = true;
  public static boolean SWERVEPOD_ENCODER_INVERTED = false;

  public static double SWERVEPOD_THRUST_REDUCTION = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static double SWERVEPOD_THRUST_CURRENTLIMIT = 80;
  public static boolean SWERVEPOD_THRUST_INVERTED = true;

 // public static double ROBOT_MASS_KG = 71.0;
 // public static double ROBOT_MOI = .0;
 // public static double WHEEL_COF= 71;

 /* 
  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
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
}
*/
public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(1.0,0,0);
            public static final PIDConstants kRotationPID = new PIDConstants(1.0,0,0);

            public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
                DriveConstants.AutoConstants.kTranslationPID, 
                DriveConstants.AutoConstants.kRotationPID
            );


            public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
                DriveConstants.AutoConstants.kTranslationPID, 
                DriveConstants.AutoConstants.kRotationPID
            );

            public static final Time kAutoAlignPredict = Seconds.of(0.0);

            public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
            public static final Distance kPositionTolerance = Centimeter.of(1.0);
            public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);

            public static final Time kEndTriggerDebounce = Seconds.of(0.1);

            public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
            public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);


            public static final LinearVelocity kStationApproachSpeed = InchesPerSecond.of(5);
            public static final Time kStationApproachTimeout = Seconds.of(5);

            public static final PathConstraints kStartingPathConstraints = new PathConstraints(3, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.


            public static final PathConstraints kPathConstraints = new PathConstraints(2, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
        
            // X = side to side, Y = away from tag
            // public static final Translation2d kTagOffset = new Translation2d(0.10, 0.55); //TODO fix based off field cad

            public static final class StationVisualizationConstants {
                    public static final Pose2d kBlueLeft = new Pose2d(0.947, 7.447, Rotation2d.fromDegrees(-50));
                    public static final Pose2d kBlueRight = new Pose2d(0.947, 0.614, Rotation2d.fromDegrees(50));
                    public static final Pose2d kRedLeft = new Pose2d(16.603, 0.614, Rotation2d.fromDegrees(130));
                    public static final Pose2d kRedRight = new Pose2d(16.603, 7.447, Rotation2d.fromDegrees(-120));
            }


  void DriveConstants() {}
}}
