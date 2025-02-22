package team3176.robot.subsystems.superstructure.climb;

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
public class Climb extends SubsystemBase {
  private static Climb instance;
  private final ClimbIO io;
  private double leftSetPoint = 0;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 1, 0, 0);
  private LoggedTunableNumber LeftClimbHeight = new LoggedTunableNumber("climbLeftHeight", 0);
  private LoggedTunableNumber AmpClimbHeight = new LoggedTunableNumber("climb/climbAmpHeight", 60);

  private Climb(ClimbIO io) {
    this.io = io;
    leftPIDController.setTolerance(1.0);
  }

  public Command stopClimb() {
    return this.runOnce(() -> io.setLeftVoltage(0.0));
  }

  private void climbGoToPosition(double position) {
    if (position > SuperStructureConstants.CLIMBLEFT_TOP_POS) {
      position = SuperStructureConstants.CLIMBLEFT_TOP_POS;
    }
    if (position < 0.0) {
      position = 0.0;
    }
    io.setLeftPIDPosition(position);
  }

  public Command setClimbPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          climbGoToPosition((position.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }


  public Command moveClimbPosition(DoubleSupplier delta) {
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
              climbGoToPosition(position.getAsDouble());
            },
            () -> {
              io.setLeftVoltage(0.0);
            })
        .until(() -> leftPIDController.atSetpoint());
  }

  public Command stow() {
    return goToPosition(() -> 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    pid.checkParemeterUpdate();
  }

  public static Climb getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Climb(new ClimbIOTalon() {});
        System.out.println("Climb instance created for Mode.REAL");
      } else {
        //instance = new Climb(new ClimbIOSim() {});
        System.out.println("Climb instance created for Mode.SIM");
      }
    }
    return instance;
  }
}
