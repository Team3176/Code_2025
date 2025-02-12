package team3176.robot.constants;

import team3176.robot.Constants;
import team3176.robot.Constants.RobotType;

/** File for storing all hardware IDs to prevent double assignments */
public class Hardwaremap {
  /*
   * Superstructer CIDs & CBNs
   */
  public static final int conveyor = 3;
  public static final int shooterWheelLeft_CID = 50;
  public static final int shooterWheelRight_CID = 6;
  public static final int shooterTransfer_CID = 5;
  public static final int shooterPivot_CID = 8;
  public static final int intakeRoller_CID = 9;
  public static final int indexerRoller_CID = 9;
  /*
  public static final int LaserCanIntakeSide_CID = 49;
  public static final int LaserCanShooterSide_CID = 48;
  */
  public static final int intakePivot_CID = 4;
  public static final int indexerPivot_CID = 4;
  public static final int climbLeft_CID = 45;
  public static final int climbRight_CID = 46;
  public static final int elevatorLeft_CID = 45;
  public static final int elevatorRight_CID = 46;
  public static int PDH_CID = 1;

  public static final String conveyor_CBN = "rio";
  public static final String shooterWheelUpper_CBN = "rio";
  public static final String shooterWheelLower_CBN = "rio";
  public static final String shooterTransfer_CBN = "rio";
  public static final String shooterPivot_CBN = "rio";
  public static final String intakeRoller_CBN = "rio";
  public static final String intakePivot_CBN = "rio";
  public static final String indexerPivot_CBN = "rio";
  /*   public static final String LaserCan_CBN = "rio"; */
  public static final String climbLeft_CBN = "rio";
  public static final String climbRight_CBN = "rio";
  public static final String elevatorLeft_CBN = "rio";
  public static final String elevatorRight_CBN = "rio";
  public static final String indexerRoller_CBN = "rio";

  public static String PDH_CBN = "rio";

  // public static final int elevatorLeaderLimitSwitch_DIO = 5;
  // public static final int elevatorFollowerLimitSwitch_DIO = 6;
  public static final int climbLBLimitSwitch_DIO = 9;
  public static final int climbRBLimitSwitch_DIO = 0; // 1;
  public static final int elevatorLBLimitSwitch_DIO = 9;
  public static final int elevatorRBLimitSwitch_DIO = 0; // 1;
  // public static final int intakeRollerLinebreak_DIO = 5;
  // public static final int intakePivotLinebreak_DIO = 4;
  public static final int intakeUpperLimitSwitch_DIO = 7;
  public static final int intakeLowerLimitSwitch_DIO = 8; // 7;
  public static final int indexerUpperLimitSwitch_DIO = 7;
  public static final int indexerLowerLimitSwitch_DIO = 8; // 7;
  public static final int shooterPivotLower_DIO = 2; // 8;
  public static final int shooterPivotUpper_DIO = 1; // 9;
  public static final int conveyorFrontLinebreak = 6;
  public static final int conveyorBackLinebreak = 4;

  public static final int blinkin_pwm_port = 9;

  /*
  
   */
  // statics constants for swerve pods
  public static final String SWERVEPOD_CTRE_CBN =
      Constants.getRobot() == RobotType.ROBOT_2025C ? "canivore" : "rio";
  public static final String SWERVEPOD_CBN = SWERVEPOD_CTRE_CBN;
  public static final SwervePodHardwareID POD001 =
      new SwervePodHardwareID(1, 10, SWERVEPOD_CBN, 11, SWERVEPOD_CBN, 12, SWERVEPOD_CBN, 81.73);
  public static final SwervePodHardwareID POD002 =
      new SwervePodHardwareID(2, 20, SWERVEPOD_CBN, 21, SWERVEPOD_CBN, 22, SWERVEPOD_CBN, 15.820);
  public static final SwervePodHardwareID POD003 =
      new SwervePodHardwareID(3, 30, SWERVEPOD_CBN, 31, SWERVEPOD_CBN, 32, SWERVEPOD_CBN, -141.54);
  public static final SwervePodHardwareID POD004 =
      new SwervePodHardwareID(
          4, 40, SWERVEPOD_CBN, 41, SWERVEPOD_CBN, 42, SWERVEPOD_CBN, 44.121); // 120.5
  public static final SwervePodHardwareID POD005 =
      new SwervePodHardwareID(5, 13, SWERVEPOD_CBN, 14, SWERVEPOD_CBN, 15, SWERVEPOD_CBN, 103.525);

  public static final String SWERVEPOD_REV_CBN = "rio";

  public static final SwervePodHardwareID FR = POD001;
  public static final SwervePodHardwareID FL = POD002;
  public static final SwervePodHardwareID BL = POD003;
  public static final SwervePodHardwareID BR = POD004;

  public static final int STEER_FR_CID = 11;
  public static final int STEER_FL_CID = 21;
  public static final int STEER_BL_CID = 31;
  public static final int STEER_BR_CID = 41;
  public static final String STEER_FR_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_FL_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_BL_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_BR_CBN = SWERVEPOD_REV_CBN;
}
