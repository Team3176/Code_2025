// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;
import team3176.robot.util.TalonUtils;

/** Template hardware interface for the Elevator subsystem. */
public class ElevatorIOTalon implements ElevatorIO {

  TalonFX elevatorLeftLeader, elevatorRightFollower;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput elevatorLBLimitswitch, elevatorRBLimitswitch;
  TalonFXConfiguration configsLeft, configsRight;
  // private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<Angle> leftPosition;
  // private final StatusSignal<Double> rightError;
  private final StatusSignal<Double> leftError;
  // private final StatusSignal<Voltage> rightVolts;
  private final StatusSignal<Voltage> leftVolts;
  // private final StatusSignal<Current> rightAmps;
  private final StatusSignal<Current> leftAmps;

  public ElevatorIOTalon() {
    configsLeft = new TalonFXConfiguration();
    // configsRight = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0);
    elevatorLBLimitswitch = new DigitalInput(Hardwaremap.elevatorLBLimitSwitch_DIO);
    elevatorRBLimitswitch = new DigitalInput(Hardwaremap.elevatorRBLimitSwitch_DIO);
    elevatorLeftLeader = new TalonFX(Hardwaremap.elevatorLeft_CID, Hardwaremap.elevatorLeft_CBN);
    elevatorRightFollower =
        new TalonFX(Hardwaremap.elevatorRight_CID, Hardwaremap.elevatorRight_CBN);
    elevatorLeftLeader.getConfigurator().apply(configsLeft);
    elevatorRightFollower.getConfigurator().apply(configsLeft);
    elevatorRightFollower.setControl(new Follower(elevatorLeftLeader.getDeviceID(), false));
    elevatorLeftLeader.setSafetyEnabled(true);
    // config setting
    configsLeft.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configsLeft.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    // configsLeft.Voltage.PeakForwardVoltage = 8;
    // configsLeft.Voltage.PeakReverseVoltage = -8;
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.ELEVATORLEFT_TOP_POS;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.ELEVATORLEFT_ZERO_POS;
    configsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // configsRight.Slot0.kP = 2.4; // An error of 1 rotations results in 40 amps output
    // configsRight.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    // configsRight.Slot0.kD = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    // configsRight.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    // configsRight.Voltage.PeakForwardVoltage = 8;
    // configsRight.Voltage.PeakReverseVoltage = -8;
    // configsRight.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // configsRight.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // SuperStructureConstants.ELEVATORRIGHT_TOP_POS;
    // configsRight.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // configsRight.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // SuperStructureConstants.ELEVATORRIGHT_ZERO_POS;
    // configsRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    TalonUtils.applyTalonFxConfigs(elevatorLeftLeader, configsLeft);
    // TalonUtils.applyTalonFxConfigs(elevatorRightFollower, configsRight);
    // elevatorLeftLeader.setInverted(true);
    // elevatorRightFollower.setInverted(false);

    leftPosition = elevatorLeftLeader.getPosition();
    // rightPosition = elevatorRightFollower.getPosition();
    // rightError = elevatorRightFollower.getClosedLoopError();
    leftError = elevatorLeftLeader.getClosedLoopError();
    // rightAmps = elevatorRightFollower.getStatorCurrent();
    leftAmps = elevatorLeftLeader.getStatorCurrent();
    // rightVolts = elevatorRightFollower.getMotorVoltage();
    leftVolts = elevatorLeftLeader.getMotorVoltage();

    elevatorRightFollower.setPosition(0);
    elevatorLeftLeader.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPosition,
        // rightPosition,
        // rightError,
        leftError,
        // rightAmps,
        leftAmps,
        // rightVolts,
        leftVolts);
    elevatorLeftLeader.optimizeBusUtilization();
    elevatorRightFollower.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        // rightPosition,
        // rightError,
        leftError,
        // rightAmps,
        leftAmps,
        // rightVolts,
        leftVolts);
    inputs.isLeftLimitswitch = (!elevatorLBLimitswitch.get());
    inputs.isRightLimitswitch = (!elevatorRBLimitswitch.get());
    inputs.leftPosition = leftPosition.getValueAsDouble();
    // inputs.rightPosition = rightPosition.getValueAsDouble();
    inputs.leftError = leftError.getValue();
    // inputs.rightError = rightError.getValue();
    // inputs.rightAmpsStator = rightAmps.getValueAsDouble();
    inputs.leftAmpsStator = leftAmps.getValueAsDouble();
    // inputs.rightVolts = rightVolts.getValueAsDouble();
    inputs.leftVolts = leftVolts.getValueAsDouble();
  }

  @Override
  public void setLeftPIDPosition(double position) {
    elevatorLeftLeader.setControl(voltPosition.withPosition(position));
  }

  // @Override
  // public void setRightPIDPosition(double position) {
  // elevatorRightFollower.setControl(voltPosition.withPosition(position));
  // }

  // @Override
  // public void setRight(double percent) {
  // elevatorRightFollower.set(percent);
  // }

  @Override
  public void setLeft(double percent) {
    elevatorLeftLeader.set(percent);
  }

  // @Override
  // public void setRightVoltage(double voltage) {
  // elevatorRightFollower.setVoltage(voltage);
  // }

  @Override
  public void setLeftVoltage(double voltage) {
    elevatorLeftLeader.setVoltage(voltage);
  }

  @Override
  public void setElevatorVoltge(double voltage) {
    elevatorLeftLeader.setVoltage(voltage);
    // elevatorRightFollower.setVoltage(voltage);
  }
  // System.out.println("ElevatorIOFalcon.set was called");
  // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
  // elevatorLeaderMotor.set(1);

}
