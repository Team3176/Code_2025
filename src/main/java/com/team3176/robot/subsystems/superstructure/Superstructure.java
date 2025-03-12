package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.playingwithfusion.TimeOfFlight;

import com.team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.superstructure.tof.TimeOfFlightIOFusion;
import com.team3176.robot.util.LoggedTunableNumber;

public class Superstructure {
  private static Superstructure instance;
  private TimeOfFlightIOFusion tof;

  public Superstructure() {
    tof = TimeOfFlightIOFusion.getInstance();
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }


 


}
