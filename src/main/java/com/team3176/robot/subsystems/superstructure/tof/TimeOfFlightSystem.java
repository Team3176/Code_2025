package com.team3176.robot.subsystems.superstructure.tof;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.team3176.robot.constants.Hardwaremap;


public class TimeOfFlightSystem extends SubsystemBase{
  private TimeOfFlightIOFusion TOF_right;
  private TimeOfFlightIOFusion TOF_left;
  private static TimeOfFlightSystem instance;

  public TimeOfFlightSystem() {
    TOF_left = new TimeOfFlightIOFusion(Hardwaremap.TOF_LEFT_CID);
    TOF_right = new TimeOfFlightIOFusion(Hardwaremap.TOF_RIGHT_CID);
  }


  public static TimeOfFlightSystem getInstance() {
    if (instance == null) {
      instance = new TimeOfFlightSystem();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
