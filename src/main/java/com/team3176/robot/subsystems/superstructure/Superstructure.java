package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.drivetrain.Drivetrain;
import com.team3176.robot.subsystems.superstructure.climb.Climb;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollers.POS;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollers;
import com.team3176.robot.subsystems.superstructure.elevator.Elevator;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Arm arm;
  private ArmRollers armrollers;
  private Elevator elevator;
  private final LoggedTunableNumber pivotTuneSetPoint, velTuneSetPoint, elevTunePositionSetPoint, climbTunePositionSetPoint;
  private final LoggedTunableNumber L1ElvSetpoint, L2ElvSetpoint, L3ElvSetpoint, L4ElvSetpoint;
  private final LoggedTunableNumber HumanLoadElvSetpoint;
  private final LoggedTunableNumber AHOMETuneSetpoint,A1TuneSetpoint, A2TuneSetpoint, A3TuneSetpoint, A4TuneSetpoint;
  private final LoggedTunableNumber A_PROCESSORTuneVolts, A_INTAKETuneVolts, A_BARGETuneVolts, C_INTAKETuneVolts, C_SSPITTuneVolts, C_FSPITTuneVolts;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
    armrollers = ArmRollers.getInstance();
    elevator = Elevator.getInstance();
    this.pivotTuneSetPoint = new LoggedTunableNumber("ss/pivotSetpoint", 0);
    this.velTuneSetPoint = new LoggedTunableNumber("ss/velSetpoint", 0);

    this.elevTunePositionSetPoint = new LoggedTunableNumber("ss/posSetpoint", 0);
    this.climbTunePositionSetPoint = new LoggedTunableNumber("ss/posSetpoint", 0);
    this.HumanLoadElvSetpoint = new LoggedTunableNumber("ss/ElvL1setpoint", 0);
    this.L1ElvSetpoint = new LoggedTunableNumber("ss/ElvL1setpoin", SuperStructureConstants.ELEVATORLEADER_L1_POS);
    this.L2ElvSetpoint = new LoggedTunableNumber("ss/ElvL2setpoin", SuperStructureConstants.ELEVATORLEADER_L2_POS);
    this.L3ElvSetpoint = new LoggedTunableNumber("ss/ElvL3setpoin", SuperStructureConstants.ELEVATORLEADER_L3_POS);
    this.L4ElvSetpoint = new LoggedTunableNumber("ss/ElvL4setpoint", SuperStructureConstants.ELEVATORLEADER_L4_POS);

    this.AHOMETuneSetpoint = new LoggedTunableNumber("ss/AHomeSetpoint", SuperStructureConstants.ARM_AHOME_POS);
    this.A1TuneSetpoint = new LoggedTunableNumber("ss/A1Setpoint", SuperStructureConstants.ARM_A1_POS);
    this.A2TuneSetpoint = new LoggedTunableNumber("ss/A2Setpoint", SuperStructureConstants.ARM_A2_POS);
    this.A3TuneSetpoint = new LoggedTunableNumber("ss/A3Setpoint", SuperStructureConstants.ARM_A3_POS);
    this.A4TuneSetpoint = new LoggedTunableNumber("ss/A4Setpoint", SuperStructureConstants.ARM_A4_POS);
    this.A_PROCESSORTuneVolts = new LoggedTunableNumber("ss/AProcessorVolts", SuperStructureConstants.ARM_A_PROCESSORVOLTS);
    this.A_INTAKETuneVolts = new LoggedTunableNumber("ss/AIntakeVolts", SuperStructureConstants.ARM_A_INTAKEVOLTS);
    this.A_BARGETuneVolts = new LoggedTunableNumber("ss/ABargeVolts", SuperStructureConstants.ARM_A_BARGEVOLTS);
    
    this.C_FSPITTuneVolts = new LoggedTunableNumber("ss/CFastSpitVolts", SuperStructureConstants.ARM_C_FASTSPITVOLTS);
    this.C_INTAKETuneVolts = new LoggedTunableNumber("ss/CIntakeVolts", SuperStructureConstants.ARM_C_INTAKEVOLTS);
    this.C_SSPITTuneVolts = new LoggedTunableNumber("ss/CSlowSPitVolts", SuperStructureConstants.ARM_C_SLOWSPITVOLTS);
  }

  public Command armVoltPos() {
    return arm.runPosition(()->this.pivotTuneSetPoint.get());
  }

  public Command arm2Home() {
    return arm.arm2Home();
  }

  public Command setPivotCoast() {
    return arm.setPivot2Coast();
  }

  public Command setPivotBrake() {
    return arm.setPivot2Brake();
  }

  public Command setClimbCoast() {
    return climb.set2Coast();
  }

  public Command setClimbBrake() {
    return climb.set2Brake();
  }

  public Command armVoltPosManual(DoubleSupplier voltage) {
    return arm.runPosition(()->this.pivotTuneSetPoint.get());
  }

  public Command armVoltVel() {
    return (armrollers.runVelocity(()-> this.velTuneSetPoint.get())).andThen(armrollers.stopRollers());
  }
public Command armVoltVelManual(DoubleSupplier voltage) { return armrollers.runVelocity(() -> voltage.getAsDouble()); }

  public Command armRevVoltVel() {
    return armrollers.runVelocity(()->-1 * this.velTuneSetPoint.get());
  }

  public Command testElevator() {
    return elevator.goToPosition(()->this.elevTunePositionSetPoint.get());
  }
  
  public Command testElevatorManual(DoubleSupplier voltage) {
    return elevator.goToPositionManual(() -> voltage.getAsDouble());
  }

  public Command goToL0() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L0_POS)).alongWith(armrollers.setPosTrack(POS.L0));
  }

  public Command goToL1() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L1_POS).alongWith(armrollers.setPosTrack(POS.L1)));
  }

  public Command goToL2() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L2_POS)).alongWith(armrollers.setPosTrack(POS.L2));
  }

  public Command goToL3() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L3_POS)).alongWith(armrollers.setPosTrack(POS.L3));
  }

  public Command goToL4() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L4_POS)).alongWith(armrollers.setPosTrack(POS.L4));
  }

  public Command goToHumanLoad() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_HF_POS).alongWith(armrollers.setPosTrack(POS.HF))); 
  }

  public Command runRollersIn () {
    return armrollers.runVelocity(() -> SuperStructureConstants.ARM_C_INTAKEVOLTS);
  }


  public Command shoot() {
    return (armrollers.shoot());
  }

  public Command stopRollers() {
    return (armrollers.stopRollers());
  }

  public Command testClimb() {
    return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }

  public Command testClimbManual(DoubleSupplier climbPosition) {
    return climb.moveClimbPosition(() -> climbPosition.getAsDouble());
  }


  public Command transStickClimbExtend() {
    return climb.moveClimbPosition(() -> 1);
  }

  public Command transStickClimbRetract() {
    return climb.moveClimbPosition(() -> -1);
  }


  /* 
  public Command getProcessorCoralLeftAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.CoralStation.leftCenterFace)
        // .andThen(Drivetrain.getInstance().chaseNote().raceWith(intakeNote()));
        .andThen(Drivetrain.getInstance().chaseNote());
  }
  */

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }


}
