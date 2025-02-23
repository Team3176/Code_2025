// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.util.TalonUtils;

/** Template hardware interface for a closed loop subsystem. */
public class ArmIOTalon implements ArmIO {

  private TalonFX rollerController;
  private TalonFX pivotController;
  private CANcoder armPivotEncoder;
  VelocityVoltage voltVelocity;
  VoltageOut rollerVolts = new VoltageOut(0.0);
  VoltageOut pivotVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private SparkClosedLoopController pivotPID;
  private Rotation2d encoderOffset; 
  
  DigitalInput rollerLinebreak;
  DigitalInput pivotLinebreak;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;

  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrentAmpsStator;
  private final StatusSignal<Current> pivotCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Angle> pivotAbsolutePosition;
  private final StatusSignal<Temperature> pivotTemp;

  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrentAmpsStator;
  private final StatusSignal<Current> rollerCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Temperature> rollerTemp;

  public ArmIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    rollerController = new TalonFX(Hardwaremap.armRoller_CID, Hardwaremap.armRoller_CBN);
    pivotController = new TalonFX(Hardwaremap.armPivot_CID, Hardwaremap.armPivot_CBN);

    armPivotEncoder = new CANcoder(Hardwaremap.armCancoder_CID, Hardwaremap.armPivot_CBN);
    //private Rotation2d offset; 

    var pivotEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.ARM_ENCODER_OFFSET);
    pivotEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    ;

    armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    pivotConfigs.Slot0.kP = 60; // An error of 1 rotation results in 2.4 V output
    pivotConfigs.Slot0.kI = 0; // No output for integrated error
    pivotConfigs.Slot0.kD = 1; // A velocity of 1 rps results in 0.1 V output

    pivotConfigs.Voltage.PeakForwardVoltage = 8;
    pivotConfigs.Voltage.PeakReverseVoltage = -10;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.armCancoder_CID;
    pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfigs.Feedback.SensorToMechanismRatio = 20.0;

    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        2;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        -2;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 

    TalonUtils.applyTalonFxConfigs(rollerController, rollerConfigs);
    TalonUtils.applyTalonFxConfigs(pivotController, pivotConfigs);
    pivotController.setPosition(0, 0);

    pivotAppliedVolts = pivotController.getMotorVoltage();
    pivotCurrentAmpsStator = pivotController.getStatorCurrent();
    pivotCurrentAmpsSupply = pivotController.getSupplyCurrent();
    pivotVelocity = pivotController.getVelocity();
    pivotPosition = pivotController.getPosition();
    pivotAbsolutePosition = armPivotEncoder.getAbsolutePosition();
    pivotTemp = pivotController.getDeviceTemp();

    rollerAppliedVolts = rollerController.getMotorVoltage();
    rollerCurrentAmpsStator = rollerController.getStatorCurrent();
    rollerCurrentAmpsSupply = rollerController.getSupplyCurrent();
    rollerVelocity = rollerController.getVelocity();
    rollerTemp = rollerController.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    rollerController.optimizeBusUtilization();
    pivotController.optimizeBusUtilization();
  }


  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply
        );
    BaseStatusSignal.refreshAll(
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    // inputs.isRollerLinebreak = (!rollerLinebreak.get());
    // inputs.isPivotLinebreak = (!pivotLinebreak.get());

    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotAmpsStator = pivotCurrentAmpsStator.getValueAsDouble();
    inputs.pivotAmpsSupply = pivotCurrentAmpsSupply.getValueAsDouble();
    inputs.pivotTempCelcius = pivotTemp.getValueAsDouble();
    inputs.pivotPosition = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());

    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAmpsStator = rollerCurrentAmpsStator.getValueAsDouble();
    inputs.rollerAmpsSupply = rollerCurrentAmpsSupply.getValueAsDouble();
    inputs.rollerTempCelcius = rollerTemp.getValueAsDouble();
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());    
    inputs.pivotAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(armPivotEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);
  }

  @Override
  public void setRollerVolts(double volts) {
    rollerController.setControl(rollerVolts.withOutput(volts));
  }

  @Override
  public void setPivotPIDPosition(double position) {
    pivotPID.setReference(position, SparkBase.ControlType.kPosition);
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotController.setControl(pivotVolts.withOutput(volts));
  }

  @Override
  public void setPivotVoltagePos(double position) {
    pivotController.setControl(voltPosition.withPosition(position));
  }
}
