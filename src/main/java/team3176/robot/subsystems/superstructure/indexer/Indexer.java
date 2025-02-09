package team3176.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID;

public class Indexer extends SubsystemBase {
  private static Indexer instance;
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final LoggedTunableNumber deployPivotVolts;
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber retractPivotVolts;
  private final LoggedTunableNumber waitTime;
  private final TunablePID pivotPID;
  private Timer deployTime = new Timer();
  private double pivotSetpoint;
  private final double DEPLOY_POS = 2.1;
  private double pivot_offset = 0;
  private InterpolatingDoubleTreeMap kG = new InterpolatingDoubleTreeMap();
  private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD
  };

  private pivotStates pivotState = pivotStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.indexerRollerLinebreak_DIO);

  private Indexer(IndexerIO io) {
    this.io = io;
    this.pivotPID = new TunablePID("indexerPivot", 3.0, 0.0, 0.0);
    this.deployPivotVolts = new LoggedTunableNumber("indexer/rollerDeployVolts", 0);
    this.rollerVolts = new LoggedTunableNumber("indexer/rollerVolts", 7.0);
    this.retractPivotVolts = new LoggedTunableNumber("indexer/rollerRetractVolts", 0);
    this.waitTime = new LoggedTunableNumber("indexer/waitTime", 0);
  }

  public Command EmergencyHold() {
    return this.runEnd(() -> io.setPivotVolts(-2.5), () -> io.setPivotVolts(0.0));
  }

  public Command manualDown() {
    return this.runEnd(
        () -> {
          io.setPivotVolts(2.5);
          io.setRollerVolts(4.0);
        },
        () -> {
          io.setPivotVolts(0.0);
          io.setRollerVolts(0);
        });
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the indexer and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  private boolean rollerSwitch() {
    return lastRollerSpeed - inputs.rollerVelocityRadPerSec > 15.0;
  }

  public static Indexer getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Indexer(new IndexerIOTalon() {});
      } else {
        instance = new Indexer(new IndexerIOSim() {});
      }
    }
    return instance;
  }

  // Example command to show how to set the pivot state
  public Command deployPivot() {
    return this.runOnce(
        () -> {
          this.pivotSetpoint = DEPLOY_POS;
          deployTime.restart();
        });
  }

  public Command retractPivot() {
    return this.runOnce(() -> this.pivotSetpoint = 0.0);
  }

  public Command climbindexer() {
    return this.runOnce(() -> this.pivotSetpoint = 0.45);
  }

  public Command spinindexer() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get()), () -> io.setRollerVolts(0));
  }

  public Command spinindexerRollersSlow() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get() / 2), () -> io.setRollerVolts(0));
  }

  public Command stopRollers() {
    return this.runOnce(() -> io.setRollerVolts(0));
  }
  /*
   * this can be much simpler than before just needs to spin the indexer and retract when done.
   * keep the high level logic up in superstructure
   */
  public Command indexerNote() {
    return (deployPivot()
        .andThen(spinindexer())
        .finallyDo(
            () -> {
              this.pivotSetpoint = 0.0;
              io.setRollerVolts(0.0);
            }));
  }

  // TODO: might need to deploy the indexer during a spit but maybe not
  public Command spit() {
    return this.runEnd(() -> io.setRollerVolts(-1.5), () -> io.setRollerVolts(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("indexer", inputs);
    Logger.recordOutput("indexer/state", pivotState);
    double pivot_pos = inputs.pivotPosition - pivot_offset;
    if (!ishomed && pivotSetpoint > 1.0) {
      pivot_pos = -3.0;
    }
    double commandVolts = pivotPID.calculate(pivot_pos, pivotSetpoint);
    if (pivot_pos <= 0.7) {
      commandVolts *= 1.6;
    }
    commandVolts = MathUtil.clamp(commandVolts, -3.5, 2.0);

    Logger.recordOutput("indexer/PID_out", commandVolts);
    Logger.recordOutput("indexer/setpoint", this.pivotSetpoint);
    Logger.recordOutput("indexer/offsetPos", pivot_pos);
    runPivot(commandVolts);
    pivotPID.checkParemeterUpdate();
    if (inputs.lowerLimitSwitch && !ishomed) {
      ishomed = true;
      pivot_offset = inputs.pivotPosition - DEPLOY_POS;
    }
    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
