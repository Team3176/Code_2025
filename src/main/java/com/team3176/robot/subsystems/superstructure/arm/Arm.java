package com.team3176.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;

public class Arm extends SubsystemBase {
  private static Arm instance;
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final LoggedTunableNumber pivotTuneSetPoint;
  private final LoggedTunableNumber AHOMETuneSetpoint,A1TuneSetpoint, A2TuneSetpoint, A3TuneSetpoint, A4TuneSetpoint;
  private final TunablePID pivotPID;
  private Timer deployTime = new Timer();
  private double pivotSetpoint;
  private double pivot_offset = 0;
  private boolean ishomed = false;
  private double pivotHome = SuperStructureConstants.ARM_AHOME_POS;
  private double AHOMESetpoint, A1Setpoint, A2Setpoint, A3Setpoint, A4Setpoint;
  public enum POS {
    HOME,
    A1,
    A2,
    A3,
    A4,
  }
  public POS currentPosTrack = POS.HOME;

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD,
  };
  

  private pivotStates pivotState = pivotStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.ArmRollerLinebreak_DIO);

  private Arm(ArmIO io) {
    this.io = io;
    this.pivotPID = new TunablePID("ArmPivot", 3.0, 0.0, 0.0);
    this.pivotTuneSetPoint = new LoggedTunableNumber("Arm/pivotSetpoint", 0);

    this.AHOMETuneSetpoint = new LoggedTunableNumber("Arm/AHOMESetpoint", SuperStructureConstants.ARM_AHOME_POS);
    this.A1TuneSetpoint = new LoggedTunableNumber("Arm/A1Setpoint", SuperStructureConstants.ARM_A1_POS);
    this.A2TuneSetpoint = new LoggedTunableNumber("Arm/A2Setpoint", SuperStructureConstants.ARM_A2_POS);
    this.A3TuneSetpoint = new LoggedTunableNumber("Arm/A3Setpoint", SuperStructureConstants.ARM_A3_POS);
    this.A4TuneSetpoint = new LoggedTunableNumber("Arm/A4Setpoint", SuperStructureConstants.ARM_A4_POS);
    
    this.pivotHome = inputs.pivotPositionRot;

    AHOMESetpoint = SuperStructureConstants.ARM_AHOME_POS;
    A1Setpoint = SuperStructureConstants.ARM_A1_POS;
    A2Setpoint = SuperStructureConstants.ARM_A2_POS;
    A3Setpoint = SuperStructureConstants.ARM_A3_POS;
    A4Setpoint = SuperStructureConstants.ARM_A4_POS;
  }

  public Command setPosTrack(POS pos){
    return this.runOnce(() -> {
      currentPosTrack = pos;
    });
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the Arm and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  public static Arm getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Arm(new ArmIOTalon() {});
      } else {
        instance = new Arm(new ArmIOSim() {});
      }
    }
    return instance;
  }

  public boolean haveCoral() {
    return inputs.hasCoral;
  }

  // Example command to show how to set the pivot state
  public Command reefLevelPivot(double reefLevel) {
    return this.runOnce(
        () -> {
          this.pivotSetpoint = reefLevel;
          deployTime.restart();
        });
  }

  public Command arm2Home() {
    return this.runOnce(
      () -> {
       setPivotVoltagePos(pivotHome); 
      }); 
    }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runPosition(DoubleSupplier position) {
    return this.run(
      () -> { 
        setPivotVoltagePos(position.getAsDouble());
      });
  }

  public Command runPositionVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setPivotVolts(position.getAsDouble());
      }, 
      () -> {
        setPivotVolts(0.0);
      });
  }


  private void setPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  private void setPivotVoltagePos(double position) {
    io.setPivotVoltagePos(position);
  }

    public void setPivotCoast() {
    io.setPivotBrakeMode(false);
  }

  public void setPivotBrake() {
    io.setPivotBrakeMode(true);
  }
  
  public Command setPivot2Coast() {
    return this.runOnce(
      () -> {
        setPivotCoast();
      }); 
    }

  public Command setPivot2Brake() {
    return this.runOnce(
      () -> {
        setPivotCoast();
      }); 
    }



  @Override
  public void periodic() {
    io.updateLaserCanMeasurement();
    io.updateInputs(inputs);



    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/state", pivotState);
    if (this.pivotTuneSetPoint.hasChanged(hashCode())){


    }

    Logger.recordOutput("Arm/setpoint", this.pivotSetpoint);
    pivotPID.checkParemeterUpdate();
  }
}
