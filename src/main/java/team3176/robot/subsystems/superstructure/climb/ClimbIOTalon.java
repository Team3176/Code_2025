// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import team3176.robot.util.TalonUtils;

/** Template hardware interface for the Elevator subsystem. */
public class ClimbIOTalon implements ClimbIO {

  TalonFX climbLeft;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput climbLBLimitswitch;
  TalonFXConfiguration configsLeft;
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<Double> leftError;
  private final StatusSignal<Voltage> leftVolts;
  private final StatusSignal<Current> leftAmps;

  public ClimbIOTalon() {
    configsLeft = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0);
    climbLBLimitswitch = new DigitalInput(Hardwaremap.climbLBLimitSwitch_DIO);
    climbLeft = new TalonFX(Hardwaremap.climbLeft_CID, Hardwaremap.climbLeft_CBN);
    // config setting
    configsLeft.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configsLeft.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.CLIMBLEFT_TOP_POS;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.CLIMBLEFT_ZERO_POS;
    configsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonUtils.applyTalonFxConfigs(climbLeft, configsLeft);
    climbLeft.setInverted(true);

    leftPosition = climbLeft.getPosition();
    leftError = climbLeft.getClosedLoopError();
    leftAmps = climbLeft.getStatorCurrent();
    leftVolts = climbLeft.getMotorVoltage();

    climbLeft.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftPosition, leftError, leftAmps, leftVolts);
    climbLeft.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftPosition, leftError, leftAmps, leftVolts);
    inputs.isLeftLimitswitch = (!climbLBLimitswitch.get());
    inputs.leftPosition = leftPosition.getValueAsDouble();
    inputs.leftError = leftError.getValue();
    inputs.leftAmpsStator = leftAmps.getValueAsDouble();
    inputs.leftVolts = leftVolts.getValueAsDouble();
  }

  @Override
  public void setLeftPIDPosition(double position) {
    climbLeft.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setLeft(double percent) {
    climbLeft.set(percent);
  }

  @Override
  public void setLeftVoltage(double voltage) {
    climbLeft.setVoltage(voltage);
  }

  @Override
  public void setClimbVoltge(double voltage) {
    climbLeft.setVoltage(voltage);
  }
}
