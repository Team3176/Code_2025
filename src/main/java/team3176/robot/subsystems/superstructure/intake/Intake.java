package team3176.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedTunableNumber deployPivotVolts;
  private final LoggedTunableNumber rollerVolts; // note to self, use as velocity
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
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final TalonFXConfiguration configs = new TalonFXConfiguration();

  /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
     configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
     configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
     configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
     configs.Slot0.kI = 0; // No output for integrated error
     configs.Slot0.kD = 0; // No output for error derivative
     // Peak output of 8 volts
  **/

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD
  };

  private pivotStates pivotState = pivotStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);

  private Intake(IntakeIO io) {
    this.io = io;
    this.pivotPID = new TunablePID("intakePivot", 3.0, 0.0, 0.0);
    this.deployPivotVolts = new LoggedTunableNumber("intake/rollerDeployVolts", 0);
    this.rollerVolts = new LoggedTunableNumber("intake/rollerVolts", 7.0);
    this.retractPivotVolts = new LoggedTunableNumber("intake/rollerRetractVolts", 0);
    this.waitTime = new LoggedTunableNumber("intake/waitTime", 0);
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
    // this assumes positive voltage deploys the intake and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  private void intakeGoToPosition(double position) {
    io.setIntakePIDPosition(position);
  }

  public Command setIntakePosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          intakeGoToPosition((position.getAsDouble()));
        },
        () -> io.setIntakeVoltage(0.0));
  }

  public Command moveIntakePosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setIntakeVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setIntakeVoltage(0.0));
  }

  private boolean rollerSwitch() {
    return lastRollerSpeed - inputs.rollerVelocityRadPerSec > 15.0;
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Intake(new IntakeIOTalon() {});
      } else {
        instance = new Intake(new IntakeIOSim() {});
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

  public void movePivot(DoubleSupplier x) {
    io.setPivotPIDPosition(x.getAsDouble());
  }

  public Command movePivotPid(DoubleSupplier position) {
    return this.runEnd(() -> movePivot(position), () -> io.setPivotVolts(0));
  }

  public void movePivotVelocity(DoubleSupplier x) {
    io.setVelocityVoltage(x.getAsDouble());
  }

  public Command movePivotVelocityVoltage(DoubleSupplier velocity) {
    return this.runEnd(() -> movePivotVelocity(velocity), () -> io.setVelocityVoltage(0));
  }

  public void moveVelocity(DoubleSupplier x) {
    io.setPivotVolts(x.getAsDouble());
  }

  public Command moveRollerVelocity(DoubleSupplier velocity) {
    System.out.println(velocity);
    return this.runEnd(() -> moveVelocity(velocity), () -> io.setPivotVolts(0));
  }

  public Command retractPivot() {
    return this.runOnce(() -> this.pivotSetpoint = 0.0);
  }

  public Command climbIntake() {
    return this.runOnce(() -> this.pivotSetpoint = 0.45);
  }

  public Command spinIntake() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get()), () -> io.setRollerVolts(0));
  }

  public Command spinIntakeRollersSlow() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get() / 2), () -> io.setRollerVolts(0));
  }

  public Command stopRollers() {
    return this.runOnce(() -> io.setRollerVolts(0));
  }
  /*
   * this can be much simpler than before just needs to spin the intake and retract when done.
   * keep the high level logic up in superstructure
   */
  public Command intakeNote() {
    return (deployPivot()
        .andThen(spinIntake())
        .finallyDo(
            () -> {
              this.pivotSetpoint = 0.0;
              io.setRollerVolts(0.0);
            }));
  }

  // TODO: might need to deploy the intake during a spit but maybe not
  public Command spit() {
    return this.runEnd(() -> io.setRollerVolts(-1.5), () -> io.setRollerVolts(0));
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/state", pivotState);
    double pivot_pos = inputs.pivotPosition - pivot_offset;
    if (!ishomed && pivotSetpoint > 1.0) {
      pivot_pos = -3.0;
    }
    double commandVolts = pivotPID.calculate(pivot_pos, pivotSetpoint);
    if (pivot_pos <= 0.7) {
      commandVolts *= 1.6;
    }
    commandVolts = MathUtil.clamp(commandVolts, -3.5, 2.0);

    Logger.recordOutput("Intake/PID_out", commandVolts);
    Logger.recordOutput("Intake/setpoint", this.pivotSetpoint);
    Logger.recordOutput("Intake/offsetPos", pivot_pos);
    // runPivot(commandVolts);
    pivotPID.checkParemeterUpdate();
    if (inputs.lowerLimitSwitch && !ishomed) {
      ishomed = true;
      pivot_offset = inputs.pivotPosition - DEPLOY_POS;
    }
    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
