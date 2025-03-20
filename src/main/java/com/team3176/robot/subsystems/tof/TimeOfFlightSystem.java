package com.team3176.robot.subsystems.tof;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.subsystems.tof.TimeOfFlightIO.TimeOfFlightIOInputs;
import org.littletonrobotics.junction.Logger;

public class TimeOfFlightSystem extends SubsystemBase{
  private static TimeOfFlightSystem instance;
  private final TimeOfFlightIO io;
  private final TimeOfFlightIOInputsAutoLogged inputs = new TimeOfFlightIOInputsAutoLogged();
  private double tof_left, tof_right, tof_center;
  private double tof_left_tolerance, tof_right_tolerance, tof_center_tolerance;
  private double tof_left_setpoint, tof_right_setpoint, tof_center_setpoint;


  public TimeOfFlightSystem(TimeOfFlightIO io) {
    this.io = io;
    io.updateInputs(inputs);
    tof_center_tolerance = 30;
    tof_left_tolerance = 30;
    tof_right_tolerance = 30;
    
    tof_center_setpoint = 550;
    tof_left_setpoint = 127;
    tof_right_setpoint = 300;
  }

  
  
  public Command getTofLeftRange() {
    return this.runOnce (() -> {io.getRangeRaw_left();});
  }
  public Command getTofRightRange() {
    return this.runOnce (() -> {io.getRangeRaw_right();});
  }
  public Command getTofCenterRange() {
    return this.runOnce (() -> {io.getRangeRaw_center();});
  }

  




  public static TimeOfFlightSystem getInstance() {
    if (instance == null) {
      instance = new TimeOfFlightSystem(new TimeOfFlightIOFusion() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TOF", inputs);
    // This method will be called once per scheduler run
  }
}
