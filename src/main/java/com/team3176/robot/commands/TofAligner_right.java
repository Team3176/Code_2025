// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.commands;

import com.team3176.robot.subsystems.drivetrain.*;
import com.team3176.robot.subsystems.tof.*;
import com.team3176.robot.subsystems.tof.TimeOfFlightSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TofAligner_right extends Command {
  /** Creates a new TofAligner_right. */

  private final TimeOfFlightSystem tofSystem = TimeOfFlightSystem.getInstance();
  private final Drive drive = Drive.getInstance();
  private double tof_tolerance = 100;  // mm (I believe)
  private double tof_left, tof_right, tof_center;
  private double tof_left_setpoint, tof_right_setpoint, tof_center_setpoint;
  private PIDController tofController = new PIDController(1.0, 0.0, 0.0);
  
  public TofAligner_right() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
