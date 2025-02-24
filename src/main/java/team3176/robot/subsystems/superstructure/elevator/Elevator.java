package team3176.robot.subsystems.superstructure.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.subsystems.superstructure.Superstructure;
import team3176.robot.util.LoggedTunableNumber;
// import team3176.robot.subsystems.superstructure.ClimbIOInputsAutoLogged;
import team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private final ElevatorIO io;
  private double leftSetPoint = 0;
  private double rightSetPoint = 0;
  private double offsetbot = 0;
 

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 1, 0, 0);
  private TunablePID rightPIDController = new TunablePID("climbRight", 1, 0, 0);
  private LoggedTunableNumber LeftClimbHeight = new LoggedTunableNumber("climbLeftHeight", 0);
  private LoggedTunableNumber AmpClimbHeight = new LoggedTunableNumber("climb/climbAmpHeight", 60);
  private LoggedTunableNumber HumanLoadTuneElvSetpoint = new LoggedTunableNumber("Elevator/L0setpoint", 0);
  private LoggedTunableNumber L1TuneElvSetpoint = new LoggedTunableNumber("Elevator/L1setpoint", 25);
  private LoggedTunableNumber L2TuneElvSetpoint = new LoggedTunableNumber("Elevator/L2setpoint", 50);
  private LoggedTunableNumber L3TuneElvSetpoint = new LoggedTunableNumber("Elevator/L3setpoint", 75);
  private LoggedTunableNumber L4TuneElvSetpoint = new LoggedTunableNumber("Elevator/L3setpoint", 107);
  private int LogSkipCounter;
  private double desiredSetpoint;
  private double HumanLoadElvSetpoint, L1ElvSetpoint, L2ElvSetpoint, L3ElvSetpoint, L4ElvSetpoint;

  

  private Elevator(ElevatorIO io) {
    this.io = io;
    leftPIDController.setTolerance(1.0);
    rightPIDController.setTolerance(1.0);
    io.updateInputs(inputs);

    System.out.println("Elevator Constructor Running");

    if ((inputs.istopLimitswitch == true )){
      System.out.println(inputs.leftPosition);
    } else {
      System.out.println("Top Limit Switch Not Pressed");
    }
   
    if ((inputs.isbotLimitswitch == true )){
      System.out.println(inputs.leftPosition);
      offsetbot = inputs.leftPosition;
    } else {
      System.out.println("Bottom Limit Swith Not Pressed");
    }

    LogSkipCounter = 0;

    this.desiredSetpoint = SuperStructureConstants.ELEVATORLEADER_L0_POS; 
    HumanLoadElvSetpoint = SuperStructureConstants.ELEVATORLEADER_L0_POS;
    L1ElvSetpoint = SuperStructureConstants.ELEVATORLEADER_L0_POS;
    L2ElvSetpoint = SuperStructureConstants.ELEVATORLEADER_L2_POS;
    L3ElvSetpoint = SuperStructureConstants.ELEVATORLEADER_L3_POS;
    L4ElvSetpoint = SuperStructureConstants.ELEVATORLEADER_L4_POS;
  }

  public Command stopLeft() {
    return this.runOnce(() -> io.setLeftVoltage(0.0));
  }

  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  public double getRightPosition() {
    return inputs.rightPosition;
  }

  public boolean getTopLimitswitch() {
    return inputs.istopLimitswitch;
  }

  public boolean getBotLimitswitch() {
    return inputs.isbotLimitswitch;
  }

//Positions for Elevator scoreing L1: ?, L2:50, L3:75, L4: 107
  private void leftGoToPosition(double position) {
    io.setLeftPIDPosition(position);
  }

  private double convertRobotPosToMotorPos(double robotPose) {
    return offsetbot + robotPose;
  }
  

  private void leftHeight(double height) {
    if (height == SuperStructureConstants.ELEVATORLEADER_L1_POS) {
      getTopLimitswitch();
    }
    if (height == SuperStructureConstants.ELEVATORLEADER_L0_POS) {
      getBotLimitswitch();

    }
  }

  public Command setLeftPosition(DoubleSupplier targetElevatorPosInRobotUnits) {
    System.out.println("setleftpos:" + targetElevatorPosInRobotUnits.getAsDouble());
    return this.runEnd(
        () -> {
          leftGoToPosition(convertRobotPosToMotorPos(targetElevatorPosInRobotUnits.getAsDouble()));
        },
        
        () -> io.setLeftVoltage(0.0));
  }

  public Command setLeftElevator( double LeftElevatorHeight) {
    return this.runEnd(
        () -> {
          leftHeight((LeftElevatorHeight));
        },
        () -> io.setLeftElevatorH(0.0));

  }

  public Command moveLeftPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setLeftVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }

  /** Given a double supplier run the PID until we reach the setpoint then end */
  public Command goToPosition(DoubleSupplier position) {
    return this.runEnd(
            () -> {
              leftGoToPosition(position.getAsDouble());
            },
            () -> {
              io.setLeftVoltage(0.0);
            })
        .until(() -> leftPIDController.atSetpoint() && rightPIDController.atSetpoint());
  }

  public Command goToPositionManual(DoubleSupplier position) {
    return this.runEnd(
       () -> {
        io.setLeftVoltage(6* position.getAsDouble());
      },
      () -> {
        io.setLeftVoltage(0.0);
      });
  }

  public Command goToL0() {
    return runOnce(() -> setDesiredSetpoint(SuperStructureConstants.ELEVATORLEADER_L0_POS));
  }

  public Command goToL1() {
    return runOnce(() -> setDesiredSetpoint(SuperStructureConstants.ELEVATORLEADER_L1_POS));
  }
  
  public Command goToL2() {
    return runOnce(() -> setDesiredSetpoint(SuperStructureConstants.ELEVATORLEADER_L2_POS));
  }

  public Command goToL3() {
    return runOnce(() -> setDesiredSetpoint(SuperStructureConstants.ELEVATORLEADER_L3_POS));
  }

  public Command goToL4() {
    return runOnce(() -> setDesiredSetpoint(SuperStructureConstants.ELEVATORLEADER_L4_POS));
  }


  public void setDesiredSetpoint(double setpoint) {
    this.desiredSetpoint = setpoint;
  }


  public Command stow() {
    return goToPosition(() -> 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (HumanLoadTuneElvSetpoint.hasChanged(hashCode())) {
      HumanLoadElvSetpoint = HumanLoadTuneElvSetpoint.get();
    }
    if (L1TuneElvSetpoint.hasChanged(hashCode())) {
      L1ElvSetpoint = L1TuneElvSetpoint.get();
    }
    if (L2TuneElvSetpoint.hasChanged(hashCode())) {
      L2ElvSetpoint = L2TuneElvSetpoint.get();
    }
    if (L3TuneElvSetpoint.hasChanged(hashCode())) {
      L2ElvSetpoint = L2TuneElvSetpoint.get();
    }
    if (L4TuneElvSetpoint.hasChanged(hashCode())) {
      L4ElvSetpoint = L4TuneElvSetpoint.get();
    }

    LogSkipCounter += 1;

    /* 
    if (LogSkipCounter >= 20) {
      System.out.println(inputs.leftPosition);
      LogSkipCounter = 0;
    }
    */

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    pid.checkParemeterUpdate();
  }

  public static Elevator getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Elevator(new ElevatorIOTalon() {});
        System.out.println("Elevator instance created for Mode.REAL");
      } else {
        System.out.println("Elevator instance created for Mode.SIM");
      }
    }
    return instance;
  }
}
