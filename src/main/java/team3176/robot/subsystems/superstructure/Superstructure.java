package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.arm.Arm;
import team3176.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Arm arm;
  private Elevator elevator;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
    elevator = Elevator.getInstance();
  }

  public Command testVoltPos() {
    return arm.runPosition(()->2);
  }

  public Command testVoltVel() {
    return arm.runVelocity(()->1);
  }

  public Command testRevVoltVel() {
    return arm.runVelocity(()->-1);
  }

  public Command testElevator() {
    return elevator.goToPosition(()->2);
  }

  public Command testClimb() {
    return climb.moveClimbPosition(() -> 1);
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
