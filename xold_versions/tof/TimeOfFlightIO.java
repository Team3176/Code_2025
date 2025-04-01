package com.team3176.robot.subsystems.tof;

import org.littletonrobotics.junction.AutoLog;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public interface TimeOfFlightIO {
  @AutoLog
  public static class TimeOfFlightIOInputs{
    public double rangeRaw_left = 0.0;
    public double rangeRaw_right= 0.0;
    public double rangeRaw_center= 0.0;
    public double rangeInches_left = 0.0; 
    public double rangeInches_right = 0.0; 
    public double rangeInches_center= 0.0; 
    public double rangeCentimeters_left = 0.0;
    public double rangeCentimeters_right = 0.0;
    public double rangeCentimeters_center = 0.0;
    public double rangeMeters_left = 0.0;
    public double rangeMeters_right = 0.0;
    public double rangeMeters_center= 0.0;

    TimeOfFlightIOInputs() {}
  }


  //TimeOfFlight TOF;
  public default void updateInputs (TimeOfFlightIOInputs inputs) {};

  public default int getID(){return 0;}; 

  public default void setViewZone_left(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {};
  public default void setViewZone_right(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {};
  public default void setViewZone_center(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {};

  public default double getRangeInches_left() {return 0.0;};
  public default double getRangeInches_right() {return 0.0;};
  public default double getRangeInches_center() {return 0.0;};

  public default double getRangeRaw_left() {return 0.0;};
  public default double getRangeRaw_right() {return 0.0;};
  public default double getRangeRaw_center() {return 0.0;};

  public default double getRangeCentimeters_left() {return 0.0;}; 
  public default double getRangeCentimeters_right() {return 0.0;}; 
  public default double getRangeCentimeters_center() {return 0.0;}; 
  
  public default double getRangeMeters_left() {return 0.0;}; 
  public default double getRangeMeters_right() {return 0.0;}; 
  public default double getRangeMeters_center() {return 0.0;}; 

  public default void close_left() {};
  public default void close_right() {};
  public default void close_center() {};

  public default void setRange_left(String sMode, double sampleTime) {};
  public default void setRange_right(String sMode, double sampleTime) {};
  public default void setRange_center(String sMode, double sampleTime) {};

}
