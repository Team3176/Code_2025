package team3176.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.MathUtil;
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

public class Arm extends SubsystemBase {
  private static Arm instance;
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber pivotTuneSetPoint;
  private final TunablePID pivotPID;
  private Timer deployTime = new Timer();
  private double pivotSetpoint;
  private final double DEPLOY_POS = 2.1;
  private double pivot_offset = 0;
  private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;

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
    this.rollerVolts = new LoggedTunableNumber("Arm/rollerVolts", 7.0);
    this.pivotTuneSetPoint = new LoggedTunableNumber("Arm/pivotSetpoint", 0);
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the Arm and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  public static Arm getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Arm(new ArmIOTalon() {});
      } else {
        instance = new Arm(new ArmIOSim() {});
      }
    }
    return instance;
  }

  // Example command to show how to set the pivot state
  public Command reefLevelPivot(double reefLevel) {
    return this.runOnce(
        () -> {
          this.pivotSetpoint = reefLevel;
          deployTime.restart();
        });
  }

  public Command stopRollers() {
    return this.runOnce(() -> io.setRollerVolts(0));
  }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runPosition(DoubleSupplier position) {
    return this.run(() -> io.setPivotVoltagePos(position.getAsDouble()));
  }

  public Command runPositionVoltageManual(DoubleSupplier position) {
    return this.runEnd(() -> io.setPivotVolts(12* position.getAsDouble()), () -> io.setPivotVolts(0.0));
  }

  public Command runVelocity(DoubleSupplier volts) {
    return this.runEnd(() -> io.setRollerVolts(12* volts.getAsDouble()), () -> io.setRollerVolts(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/state", pivotState);
    if (this.pivotTuneSetPoint.hasChanged(hashCode())){


    }
    double pivot_pos = inputs.pivotPosition - pivot_offset;
    if (!ishomed && pivotSetpoint > 1.0) {
      pivot_pos = -3.0;
    }
    double commandVolts = pivotPID.calculate(pivot_pos, pivotSetpoint);
    if (pivot_pos <= 0.7) {
      commandVolts *= 1.6;
    }
    commandVolts = MathUtil.clamp(commandVolts, -3.5, 2.0);

    Logger.recordOutput("Arm/PID_out", commandVolts);
    Logger.recordOutput("Arm/setpoint", this.pivotSetpoint);
    Logger.recordOutput("Arm/offsetPos", pivot_pos);
    // runPivot(commandVolts);
    pivotPID.checkParemeterUpdate();
    if (inputs.lowerLimitSwitch && !ishomed) {
      ishomed = true;
      pivot_offset = inputs.pivotPosition - DEPLOY_POS;
    }
    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
