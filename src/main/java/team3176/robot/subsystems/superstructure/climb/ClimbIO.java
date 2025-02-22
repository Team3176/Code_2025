// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for the Elevator subsystem. */
public interface ClimbIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPosition = 0.0;
    public double leftError = 0.0;
    public double leftVolts = 0.0;
    public double leftAmpsStator = 0.0;
    public boolean isLeftLimitswitch = true;

    // constructor if needed for some inputs
    ClimbIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setLeft(double percentOutput) {}

  public default void setLeftPIDPosition(double rotations) {}

  public default void setClimbVoltage(double voltage) {}

  public default void setLeftVoltage(double voltage) {}

  public default void setClimbVoltge(double voltage) {}

  public default void reset() {}
}
