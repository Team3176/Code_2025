package team3176.robot.commands;

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.subsystems.superstructure.arm.Arm;
import team3176.robot.subsystems.superstructure.arm.ArmIO;


public class ArmStartingPosChar extends Command {

  private final Arm arm;
  public double position;
  public double Currentposition;

  public ArmStartingPosChar(Arm arm) {
    this.arm = arm;

  }


  @Override
  public void execute() {
    while (Currentposition != position) {
        if (Currentposition < position) {
            arm.testVoltVelocity(0.01);
        }
        else {
            arm.testVoltVelocity(-0.01);
        }
    }
    arm.testVoltVelocity(0);
  }

  @Override
  public boolean isFinished() {
    if (Currentposition == position) {
        return true;
    }
    return false;
  }


}

