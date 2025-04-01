// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.constants;

import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
//import com.team3176.robot.util.Structures.PIDFConstants;

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


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DynamicsConstants {  

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }


 
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
        

    

   
/* 
    public static final class OrientTowardsNearestPOIConstants {
        public static final Rotation2d REEF_OFFSET = Rotation2d.k180deg;
        public static final Translation2d REEF_CENTER_RED = new Translation2d(13.067, 4.031);
        public static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.471, 4.031);
        public static final double CORAL_STATION_ANGLE = 55;
        public static final Rotation2d BARGE_ROTATION = Rotation2d.kCCW_90deg;
        public static final Translation2d[] BARGE_RED_CAGE_POSITIONS = {
            new Translation2d(8.8, 3),
            new Translation2d(8.8, 1.9),
            new Translation2d(8.8, .8)
        };
        public static final Translation2d[] BARGE_BLUE_CAGE_POSITIONS = {
            new Translation2d(8.8, 7.2),
            new Translation2d(8.8, 6.1),
            new Translation2d(8.8, 5)
        };
    }
*/
  
        public static final class StdDevConstants {
            public static final class MegaTag1 {
                public static final double kInitialValue = 0.3;
                public static final double kTagCountReward = 0.15;
                public static final double kAverageDistancePunishment = 0.1;
                public static final double kRobotSpeedPunishment = 0.15;
                public static final double kSingleTagPunishment = 0.3;
            }
            public static final class MegaTag2 {
                public static final double kInitialValue = 0.2;
                public static final double kAverageDistancePunishment = 0.075;
                public static final double kRobotSpeedPunishment = 0.25;
                public static final double kMultipleTagsBonus = 0.05;
            }
        }

 
/* 
    public static final class DynamicsConstants {
        public static final Angle kArmAngleTolerance = Degrees.of(1);
        public static final double kElevatorHeightTolerance = Inches.of(1).in(Meters);

        public static final double kSafeElevHeightForSwerve = 0.4;


        public static final Angle kSafeArmAngle = Degrees.of(90); //TODO this is currently straight up, this might change
        public static final Angle kMoveableArmAngle = Degrees.of(83.801389); //used in cos math, so this is equivalent to ~80 degrees either side of the left horizon //TODO this is currently straight up, this might change

        public static final Angle kRemoveAlgaeArmAngle = Degrees.of(11.6);
    
        public static final double kMinSafeElevHeight = 0.385; //previously 4.361// height of the elevator for when the arm is stowed and needs to move

        public static final double kScoreLaserCanDebounce = 0.1; //seconds

        public static final int kFunnelLaserCanID = 20;
        public static final Distance funnelLCTriggerDist = Meters.of(0.2);

    }
*/
}
    
  