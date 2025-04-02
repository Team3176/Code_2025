// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.commands;

import com.team3176.robot.subsystems.drivetrain.Drive;
import com.team3176.robot.subsystems.vision.Vision;
import com.team3176.robot.constants.ReefScapeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.Waypoint;
import java.util.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {
  /** Creates a new AlignReef. */
  Pose2d targetPose, currentPose, targetFacePose;
  boolean isdone = false;
  int fiducialID = 0;
  int targetFaceID;
  PathPlannerPath path;
  PathConstraints constraints;
  Pose2d facePose[] = new Pose2d[23]; // Array to hold the face poses for reefs 17-22
  double distance[] = new double[23];

  public static enum TargetLoc{
    LEFT,
    CENTER,
    RIGHT
  } 
  TargetLoc targetLoc; 
  
  public AlignReef(TargetLoc targetLoc) {
    System.out.println("Creating AlignReef Command");
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetLoc = targetLoc; 
    Drive drive = Drive.getInstance();
    for (int i = 17 ; i < 23 ; i++) {facePose[i] = new Pose2d();}
  }

  private Pose2d getClosestReefFacePose() {
    Pose2d closestFacePose = new Pose2d();
    double distance[] = new double[23]; // Array to hold distances from bot to reef faces 17-22
    facePose[17] = ReefScapeConstants.REEFFACE_17_POSE;
    facePose[18] = ReefScapeConstants.REEFFACE_18_POSE;
    facePose[19] = ReefScapeConstants.REEFFACE_19_POSE;
    facePose[20] = ReefScapeConstants.REEFFACE_20_POSE;
    facePose[21] = ReefScapeConstants.REEFFACE_21_POSE;
    facePose[22] = ReefScapeConstants.REEFFACE_22_POSE;
    for(int i = 17; i < 23; i++) {
      //calc distance from bot to faces
      distance[i] = Math.sqrt(Math.pow((this.currentPose.getTranslation().getX() - facePose[i].getTranslation().getX()),2) + 
                Math.pow((this.currentPose.getTranslation().getY() - facePose[i].getTranslation().getY()),2 ));
    }
    double min = 100;
    double dist = 100;
    for(int i = 17; i < 23; i++) {
      if (distance[i] < dist) {dist = distance[i]; min = i; closestFacePose = facePose[i]; }
    }
    return closestFacePose;
  }

  private int getClosestReefFaceID() {
    Pose2d closestFacePose = new Pose2d();
    int min;
    double dist;
    facePose[17] = ReefScapeConstants.REEFFACE_17_POSE;
    facePose[18] = ReefScapeConstants.REEFFACE_18_POSE;
    facePose[19] = ReefScapeConstants.REEFFACE_19_POSE;
    facePose[20] = ReefScapeConstants.REEFFACE_20_POSE;
    facePose[21] = ReefScapeConstants.REEFFACE_21_POSE;
    facePose[22] = ReefScapeConstants.REEFFACE_22_POSE;
    for(int i = 17; i < 23; i++) {
      //calc distance from bot to faces
      distance[i] = Math.sqrt(Math.pow((this.currentPose.getTranslation().getX() - facePose[i].getTranslation().getX()),2) + 
                Math.pow((this.currentPose.getTranslation().getY() - facePose[i].getTranslation().getY()),2 ));
    }
    
    min = 100;
    dist = 100;
    for(int i = 17; i < 23; i++) {
      if (distance[i] < dist) {dist = distance[i]; min = i; closestFacePose = facePose[i]; }
    }
    return min;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing AlignReef Command");
    this.currentPose = Drive.getInstance().getPose();
    this.targetFaceID = getClosestReefFaceID();
    if (this.targetLoc == TargetLoc.LEFT) {
      switch(this.targetFaceID) {
        case 17: this.targetPose = ReefScapeConstants.STALK_C_POSE;
        case 18: this.targetPose = ReefScapeConstants.STALK_A_POSE;
        case 19: this.targetPose = ReefScapeConstants.STALK_K_POSE;
        case 20: this.targetPose = ReefScapeConstants.STALK_I_POSE;
        case 21: this.targetPose = ReefScapeConstants.STALK_G_POSE;
        case 22: this.targetPose = ReefScapeConstants.STALK_E_POSE;
      }
    }
    if (this.targetLoc == TargetLoc.RIGHT) {
      switch(this.targetFaceID) {
        case 17: this.targetPose = ReefScapeConstants.STALK_D_POSE;
        case 18: this.targetPose = ReefScapeConstants.STALK_B_POSE;
        case 19: this.targetPose = ReefScapeConstants.STALK_L_POSE;
        case 20: this.targetPose = ReefScapeConstants.STALK_J_POSE;
        case 21: this.targetPose = ReefScapeConstants.STALK_H_POSE;
        case 22: this.targetPose = ReefScapeConstants.STALK_F_POSE;
      }
    }
    if (this.targetLoc == TargetLoc.CENTER) {
      switch(this.targetFaceID) {
        case 17: this.targetPose = ReefScapeConstants.REEFFACE_17_POSE;
        case 18: this.targetPose = ReefScapeConstants.REEFFACE_18_POSE;
        case 19: this.targetPose = ReefScapeConstants.REEFFACE_19_POSE;
        case 20: this.targetPose = ReefScapeConstants.REEFFACE_20_POSE;
        case 21: this.targetPose = ReefScapeConstants.REEFFACE_21_POSE;
        case 22: this.targetPose = ReefScapeConstants.REEFFACE_22_POSE;
      }
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      this.currentPose,
      this.targetPose
    );

    this.constraints = new PathConstraints(
      3.0, // max velocity (m/s)
      1.0, // max acceleration (m/s^2)
      Math.toRadians(540),
      Math.toRadians(720)
    );

    this.path = new PathPlannerPath(
      waypoints,
      this.constraints,
      null,
      new GoalEndState(0,this.targetPose.getRotation())
    );


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing AlignReef Command");
    // Get the current pose of the robot
    currentPose = Drive.getInstance().getPose();
    AutoBuilder.pathfindThenFollowPath(this.path, this.constraints);
    // Get the target pose from the vision system
    //targetPose = 

    // Calculate the difference between the current and target poses
    //Translation2d translation = targetPose.getTranslation().minus(currentPose.getTranslation());
    //Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation());

    // Create a Twist2d object to represent the difference
    //Twist2d twist = new Twist2d(translation.getX(), translation.getY(), rotation.getRadians());
    
    // Use the twist to drive the robot
    //Drive.getInstance().drive(twist);
    

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending AlignReef Command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Checking if AlignReef Command is finished");
    if (currentPose == targetPose) {
      isdone = true;
    } else { isdone = false;}
    return isdone;
  }


}