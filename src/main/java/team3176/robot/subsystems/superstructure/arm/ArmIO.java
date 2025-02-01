// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {
    public double pivotPosition = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotAmpsStator = 0.0;
    public double pivotAmpsSupply = 0.0;
    public double pivotTempCelcius = 0.0;

    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerAmpsStator = 0.0;
    public double rollerAmpsSupply = 0.0;
    public double rollerTempCelcius = 0.0;

    public boolean coralLaserCAN = false;
    public boolean isPivotLinebreak = false;
    public boolean upperLimitSwitch = false;
    public boolean lowerLimitSwitch = false;

    // constructor if needed for some inputs
    ArmIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setRollerVolts(double volts) {}

  public default void setRollerPIDVelocity(double rpm) {}

  public default void setPivotVolts(double volts) {}

  public default void setPivotPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setPivotTorquePosition(double position) {}
  
}
