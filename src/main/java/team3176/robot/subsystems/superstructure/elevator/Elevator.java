package team3176.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
// import team3176.robot.subsystems.superstructure.ClimbIOInputsAutoLogged;
import team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private final ElevatorIO io;
  private double leftSetPoint = 0;
  private double rightSetPoint = 0;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 1, 0, 0);
  private TunablePID rightPIDController = new TunablePID("climbRight", 1, 0, 0);
  private LoggedTunableNumber LeftClimbHeight = new LoggedTunableNumber("climbLeftHeight", 0);
  private LoggedTunableNumber RightClimbHeight = new LoggedTunableNumber("climbRightHeight", 0);
  private LoggedTunableNumber LeftRightClimbHeight =
      new LoggedTunableNumber("climbLeftRightHeight", 0);
  private LoggedTunableNumber AmpClimbHeight = new LoggedTunableNumber("climb/climbAmpHeight", 60);

  private Elevator(ElevatorIO io) {
    this.io = io;
    leftPIDController.setTolerance(1.0);
    rightPIDController.setTolerance(1.0);
  }

  public Command stopLeft() {
    return this.runOnce(() -> io.setLeftVoltage(0.0));
  }

  //  public Command stopRight() {
  // return this.runOnce(() -> io.setRightVoltage(0.0));
  // }

  public Command stopLeftRight() {
    return this.runOnce(
        () -> {
          io.setLeftVoltage(0.0);
          // io.setRightVoltage(0.0);
        });
  }

  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  public double getRightPosition() {
    return inputs.rightPosition;
  }
  /*
  public Command leftGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setLeft(pid.calculate(getLeftPosition(), position));
          System.out.println(position);
        },
        io::stopLeft);
  }
  */
  private void leftGoToPosition(double position) {
    if (position > SuperStructureConstants.CLIMBLEFT_TOP_POS) {
      position = SuperStructureConstants.CLIMBLEFT_TOP_POS;
    }
    if (position < 0.0) {
      position = 0.0;
    }
    io.setLeftPIDPosition(position);
  }

  // private void rightGoToPosition(double position) {
  // if (position > SuperStructureConstants.CLIMBRIGHT_TOP_POS) {
  // position = SuperStructureConstants.CLIMBRIGHT_TOP_POS;
  // } else if (position < 0.0) {
  // position = 0.0;
  // }
  // io.setRightPIDPosition(position);
  // }

  public Command setLeftPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          leftGoToPosition((position.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }

  // public Command setRightPosition(DoubleSupplier position) {
  // return this.runEnd(
  //  () -> {
  //  rightGoToPosition((position.getAsDouble()));
  //  },
  // () -> io.setRightVoltage(0.0));
  // }

  public Command setAmpPosition() {
    return goToPosition(() -> AmpClimbHeight.get());
  }

  // public Command moveRightPosition(DoubleSupplier delta) {
  // return this.runEnd(
  //  () -> {
  //  io.setRightVoltage((5 * delta.getAsDouble()));
  // },
  // () -> io.setRightVoltage(0.0));
  // }

  public Command moveLeftPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setLeftVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }

  public Command moveLeftRightPositionManual(DoubleSupplier deltaLeft) {
    return this.runEnd(
        () -> {
          //        io.setRightVoltage(5 * deltaRight.getAsDouble());
          io.setLeftVoltage(5 * deltaLeft.getAsDouble());
        },
        () -> {
          //      io.setRightVoltage(0.0);
          io.setLeftVoltage(0.0);
        });
  }

  public Command moveLeftRightPosition(double LX_Pos) {
    return this.runEnd(
        () -> {
          //        io.setRightVoltage(5 * deltaRight.getAsDouble());
          io.setLeftVoltage(5 * LX_Pos);
        },
        () -> {
          //      io.setRightVoltage(0.0);
          io.setLeftVoltage(0);
        });
  }

  public Command moveLeftRightPositionTorque(double LX_Pos) {
    return this.runEnd(
        () -> {
          //        io.setRightVoltage(5 * deltaRight.getAsDouble());
          io.setLeftPositionTorque(5 * LX_Pos);
        },
        () -> {
          //      io.setRightVoltage(0.0);
          io.setLeftPositionTorque(0);
        });
  }


  /** Given a double supplier run the PID until we reach the setpoint then end */
  public Command goToPosition(DoubleSupplier position) {
    return this.runEnd(
            () -> {
              leftGoToPosition(position.getAsDouble());
              // rightGoToPosition(position.getAsDouble());
            },
            () -> {
              io.setLeftVoltage(0.0);
              // io.setRightVoltage(0.0);
            })
        .until(() -> leftPIDController.atSetpoint() && rightPIDController.atSetpoint());
  }

  public Command stow() {
    return goToPosition(() -> 0.0);
  }

  /*   public Command rightGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setRight(pid.calculate(getRightPosition(), position));
        },
        io::stopRight);
  } */

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // if (> 0) {
    // if(inputs.istopLimitswitch){
    // io.setLeftVoltage(0);
    // }
    // }
    System.out.println(inputs.leftPosition);
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
        instance = new Elevator(new ElevatorIOSim() {});
        System.out.println("Elevator instance created for Mode.SIM");
      }
    }
    return instance;
  }

  public Command moveLeftPosition(invalid i, invalid j) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'moveLeftPosition'");
  }

  public Command moveLeftPosition(int i) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'moveLeftPosition'");
  }

  public Command moveLeftRightPosition(Object deltaLeft, Object deltaRight) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'moveLeftRightPosition'");
  }
}
