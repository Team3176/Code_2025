package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.arm.Arm;
import team3176.robot.subsystems.superstructure.elevator.Elevator;
import team3176.robot.util.LoggedTunableNumber;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Arm arm;
  private Elevator elevator;
  private final LoggedTunableNumber pivotTuneSetPoint, velTuneSetPoint, elevTunePositionSetPoint, climbTunePositionSetPoint;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
    elevator = Elevator.getInstance();
    this.pivotTuneSetPoint = new LoggedTunableNumber("Arm/pivotSetpoint", 0);
    this.velTuneSetPoint = new LoggedTunableNumber("Arm/velSetpoint", 0);
    this.elevTunePositionSetPoint = new LoggedTunableNumber("Elevator/posSetpoint", 0);
    this.climbTunePositionSetPoint = new LoggedTunableNumber("climb/posSetpoint", 0);
  }

  public Command testVoltPos() {
    return arm.runPosition(()->this.pivotTuneSetPoint.get());
  }

  public Command testVoltPosManual(DoubleSupplier voltage) {
    return arm.runPosition(voltage);
  }

  public Command testVoltVel() {
    return arm.runVelocity(()-> this.velTuneSetPoint.get());
  }

  public Command testVoltVelManual(DoubleSupplier voltage) {
    return arm.runVelocity(voltage);
  }

  public Command testRevVoltVel() {
    return arm.runVelocity(()->-1 * this.velTuneSetPoint.get());
  }

  public Command testElevator() {
    return elevator.goToPosition(()->this.elevTunePositionSetPoint.get());
  }
  
  public Command testElevatorManual(DoubleSupplier voltage) {
    return elevator.goToPositionManual(voltage);
  }


  public Command testClimb(DoubleSupplier climbPosition) {
    return climb.moveClimbPosition(climbPosition);
    //return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }

  /* 
  public Command getProcessorCoralLeftAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.CoralStation.leftCenterFace)
        // .andThen(Drivetrain.getInstance().chaseNote().raceWith(intakeNote()));
        .andThen(Drivetrain.getInstance().chaseNote());
  }
  */

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }


}
