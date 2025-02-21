package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.arm.Arm;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  public Arm arm;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
  }

  public Command grabCoral() {
    return arm.spinArm();
  }

  public Command testVoltPos() {
    return arm.testVoltage();
  }

  public Command testVoltVel() {
    return arm.testVoltVelocity(1);
  }

  public Command testRevVoltVel() {
    return arm.testVoltVelocity(-1);
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
