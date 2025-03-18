package com.team3176.robot.subsystems.tof;
import com.ctre.phoenix6.BaseStatusSignal;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.team3176.robot.constants.Hardwaremap;


public class TimeOfFlightIOFusion implements TimeOfFlightIO {
  private static TimeOfFlightSystem instance;
  TimeOfFlight TOF_left, TOF_right;


  public TimeOfFlightIOFusion() {
    TOF_left = new TimeOfFlight(Hardwaremap.TOF_LEFT_CID);
    TOF_right = new TimeOfFlight(Hardwaremap.TOF_RIGHT_CID);
  }

  public void setViewZone_right(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    TOF_right.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  public void setViewZone_left(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    TOF_left.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

   public double getRangeInches_right() {
    return Math.round(((getRangeRaw_right())/(25.4))*100.0)/100.0;
  }

   public double getRangeInches_left() {
    return Math.round(((getRangeRaw_left())/(25.4))*100.0)/100.0;
  }

  public double getRangeRaw_right() {
    System.out.println("Tof getRangeRaw right = "+ TOF_right.getRange());
    return TOF_right.getRange() - 30.0;
  }

  public double getRangeRaw_left() {
    System.out.println("Tof getRangeRaw left = "+ TOF_left.getRange());
    return TOF_left.getRange() - 30.0;
  }

  public double getRangeCentimeters_right() {
    return getRangeRaw_right()/10;
  } 

  public double getRangeCentimeters_left() {
    return getRangeRaw_left()/10;
  } 

  public void close_right() {
    TOF_right.close();
  }

  public void close_left() {
    TOF_left.close();
  }

  public void setRange_right(String sMode, double sampleTime) {
    switch(sMode.toLowerCase()) {
      case "short":
       TOF_right.setRangingMode(RangingMode.Short, sampleTime);
      case "long":
       TOF_right.setRangingMode(RangingMode.Long, sampleTime);
      case "medium":
       TOF_right.setRangingMode(RangingMode.Medium, sampleTime);
    }
  }

  public void setRange_left(String sMode, double sampleTime) {
    switch(sMode.toLowerCase()) {
      case "short":
       TOF_left.setRangingMode(RangingMode.Short, sampleTime);
      case "long":
       TOF_left.setRangingMode(RangingMode.Long, sampleTime);
      case "medium":
       TOF_left.setRangingMode(RangingMode.Medium, sampleTime);
    }
  }
  @Override
  public void updateInputs(TimeOfFlightIOInputs inputs) {
    inputs.rangeRaw_right = getRangeRaw_right();
    inputs.rangeInches_right = getRangeInches_right();
    inputs.rangeCentimeters_right = getRangeCentimeters_right();
    inputs.rangeMeters_right = getRangeMeters_right();
    inputs.rangeRaw_left = getRangeRaw_left();
    inputs.rangeInches_left = getRangeInches_left();
    inputs.rangeCentimeters_left = getRangeCentimeters_left();
    inputs.rangeMeters_left = getRangeMeters_left();
  }

}
