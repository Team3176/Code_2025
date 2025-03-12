package com.team3176.robot.subsystems.superstructure.tof;

import com.team3176.robot.constants.Hardwaremap;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public class TimeOfFlightSystem {
  private TimeOfFlight TOF_right;
  private TimeOfFlight TOF_left;
  private static TimeOfFlightSystem instance;

  public TimeOfFlightSystem() {
    TOF_left = new TimeOfFlight(Hardwaremap.TOF_LEFT_CID);
    TOF_right = new TimeOfFlight(Hardwaremap.TOF_RIGHT_CID);
  }

  public void setViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    TOF_left.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
    TOF_right.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

   public double getRangeInchesLeft() {
    return Math.round(((getRangeRawLeft())/(25.4))*100.0)/100.0;
  }

   public double getRangeInchesRight() {
    return Math.round(((getRangeRawRight())/(25.4))*100.0)/100.0;
  }

  public double getRangeRawLeft() {
    return TOF_left.getRange() - 30.0;
  }

  public double getRangeRawRight() {
    return TOF_right.getRange() - 30.0;
  }


  public void close() {
    TOF_left.close();
    TOF_right.close();
  }

  public void setRange(String sMode, double sampleTime) {
    switch(sMode.toLowerCase()) {
      case "short":
       TOF_left.setRangingMode(RangingMode.Short, sampleTime);
       TOF_right.setRangingMode(RangingMode.Short, sampleTime);
      case "long":
       TOF_left.setRangingMode(RangingMode.Long, sampleTime);
       TOF_right.setRangingMode(RangingMode.Long, sampleTime);
      case "medium":
       TOF_left.setRangingMode(RangingMode.Medium, sampleTime);
       TOF_right.setRangingMode(RangingMode.Medium, sampleTime);
    }
  }

    public static TimeOfFlightSystem getInstance() {
    if (instance == null) {
      instance = new TimeOfFlightSystem();
      System.out.println("Superstructure: TOF systems instance created.");
    }
    return instance;
  }
}
