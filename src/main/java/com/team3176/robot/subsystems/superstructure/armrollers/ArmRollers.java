package com.team3176.robot.subsystems.superstructure.armrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollersIOInputsAutoLogged;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;

public class ArmRollers extends SubsystemBase {
  private static ArmRollers instance;
  private final ArmRollersIO io;
  private final ArmRollersIOInputsAutoLogged inputs = new ArmRollersIOInputsAutoLogged();
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber A_PROCESSORTuneVolts, A_INTAKETuneVolts, A_BARGETuneVolts, C_INTAKETuneVolts, C_SSPITTuneVolts, C_FSPITTuneVolts, C_L0TuneVolts, C_L1TuneVolts, C_L2TuneVolts, C_L3TuneVolts, C_L4TuneVolts; 
  private Timer deployTime = new Timer();
  private double A_PROCESSORVolts, A_BARGEVolts, A_INTAKEVolts, C_INTAKEVolts, C_SSPITVolts, C_FSPITVolts, C_L0Volts, C_L1Volts, C_L2Volts, C_L3Volts, C_L4Volts; 
  public enum POS {
    HF,
    L0,
    L1,
    L2,
    L3,
    L4,
  }
  public POS currentPosTrack = POS.L0;


  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.ArmRollerLinebreak_DIO);

  private ArmRollers(ArmRollersIO io) {
    this.io = io;
    this.rollerVolts = new LoggedTunableNumber("Arm/rollerVolts", 7.0);
    this.A_PROCESSORTuneVolts = new LoggedTunableNumber("Arm/AProcessorVolts", SuperStructureConstants.ARM_A_PROCESSORVOLTS);
    this.A_INTAKETuneVolts = new LoggedTunableNumber("Arm/AIntakeVolts", SuperStructureConstants.ARM_A_INTAKEVOLTS);
    this.A_BARGETuneVolts = new LoggedTunableNumber("Arm/ABargeVolts", SuperStructureConstants.ARM_A_BARGEVOLTS);
    this.C_FSPITTuneVolts = new LoggedTunableNumber("Arm/CFastSpitVolts", SuperStructureConstants.ARM_C_FASTSPITVOLTS);
    this.C_INTAKETuneVolts = new LoggedTunableNumber("Arm/CIntakeVolts", SuperStructureConstants.ARM_C_INTAKEVOLTS);
    this.C_SSPITTuneVolts = new LoggedTunableNumber("Arm/CSlowSPitVolts", SuperStructureConstants.ARM_C_SLOWSPITVOLTS);
    this.C_L0TuneVolts = new LoggedTunableNumber("Arm/L0Volts", SuperStructureConstants.ARM_C_L0);
    this.C_L1TuneVolts = new LoggedTunableNumber("Arm/L1Volts", SuperStructureConstants.ARM_C_L1);
    this.C_L2TuneVolts = new LoggedTunableNumber("Arm/L2Volts", SuperStructureConstants.ARM_C_L2);
    this.C_L3TuneVolts = new LoggedTunableNumber("Arm/L3Volts", SuperStructureConstants.ARM_C_L3);
    this.C_L4TuneVolts = new LoggedTunableNumber("Arm/L4Volts", SuperStructureConstants.ARM_C_L4);

    A_PROCESSORVolts = SuperStructureConstants.ARM_A_PROCESSORVOLTS;
    A_INTAKEVolts = SuperStructureConstants.ARM_A_INTAKEVOLTS;
    A_BARGEVolts = SuperStructureConstants.ARM_A_BARGEVOLTS;
    C_FSPITVolts = SuperStructureConstants.ARM_C_FASTSPITVOLTS;
    C_INTAKEVolts = SuperStructureConstants.ARM_C_INTAKEVOLTS;
    C_SSPITVolts = SuperStructureConstants.ARM_C_SLOWSPITVOLTS;
    C_L0Volts = SuperStructureConstants.ARM_C_L0;
    C_L1Volts = SuperStructureConstants.ARM_C_L1;
    C_L2Volts = SuperStructureConstants.ARM_C_L2;
    C_L3Volts = SuperStructureConstants.ARM_C_L3;
    C_L4Volts = SuperStructureConstants.ARM_C_L4;
  }

  public Command setPosTrack(POS pos){
    return this.runOnce(() -> {
      currentPosTrack = pos;
    });
  }


  public static ArmRollers getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new ArmRollers(new ArmRollersIOTalon() {});
      } else {
        instance = new ArmRollers(new ArmRollersIOSim() {});
      }
    }
    return instance;
  }

  public boolean haveCoral() {
    return inputs.hasCoral;
  }


  public Command stopRollers() {
    return this.runOnce(() -> {setRollerVolts(0.0);});
  }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runVelocity(DoubleSupplier volts) {
    return this.runEnd(
      () -> {
        setRollerVolts(volts.getAsDouble());
      }, 
      () -> {
        setRollerVolts(0.0);
      });
  }

  public Command shoot() {
    return this.run(
      () -> {
        runShoot();
      });
  }

  private void runShoot() {
    switch (currentPosTrack) {
      case L0:
        setRollerVolts(SuperStructureConstants.ARM_C_L0);
        break;
      case L1:
        setRollerVolts(SuperStructureConstants.ARM_C_L1);
        break;
      case L2:
        setRollerVolts(SuperStructureConstants.ARM_C_L2);
        break;
      case L3:
        setRollerVolts(SuperStructureConstants.ARM_C_L3);
        break;
      case L4:
        setRollerVolts(SuperStructureConstants.ARM_C_L4);
        break;
    }
  }

  public Command runRollersIn(DoubleSupplier volts) {
    return this.runOnce(
      () -> {
        setRollerVolts(volts.getAsDouble());
      });
  }



  private void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

    public void setCoast() {
    io.setBrakeMode(false);
  }

  public void setBrake() {
    io.setBrakeMode(true);
  }
  
  public Command set2Coast() {
    return this.runOnce(
      () -> {
        setCoast();
      }); 
    }

  public Command set2Brake() {
    return this.runOnce(
      () -> {
        setCoast();
      }); 
    }



  @Override
  public void periodic() {
    io.updateLaserCanMeasurement();
    io.updateInputs(inputs);

    if (C_INTAKETuneVolts.hasChanged(hashCode())) {
      C_INTAKEVolts = C_INTAKETuneVolts.get();
    }
    if (C_L0TuneVolts.hasChanged(hashCode())) {
      C_L0Volts = C_L0TuneVolts.get();
    }
    if (C_L1TuneVolts.hasChanged(hashCode())) {
      C_L1Volts = C_L1TuneVolts.get();
    }
    if (C_L2TuneVolts.hasChanged(hashCode())) {
      C_L2Volts = C_L2TuneVolts.get();
    }
    if (C_L3TuneVolts.hasChanged(hashCode())) {
      C_L3Volts = C_L3TuneVolts.get();
    }
    if (C_L4TuneVolts.hasChanged(hashCode())) {
      C_L4Volts = C_L4TuneVolts.get();
    }

    Logger.processInputs("Arm", inputs);
  }
}
