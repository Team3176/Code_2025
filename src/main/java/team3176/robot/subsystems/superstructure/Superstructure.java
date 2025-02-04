package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.arm.Arm;

public class Superstructure {
  private static Superstructure instance;
  public Arm arm;

  public Superstructure() {
    arm = Arm.getInstance();
  }

  public Command grabCoral() {
    return arm.spinArm();
  }

  public Command testPID() {
    return arm.PID();
  }

  public Command negativeTestPID() {
    return arm.negativePID();
  }

  public Command testTorque() {
    return arm.torquePosition();
  }
  
  public Command getProcessorCoralLeftAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.CoralStation.leftCenterFace)
        // .andThen(Drivetrain.getInstance().chaseNote().raceWith(intakeNote()));
        .andThen(Drivetrain.getInstance().chaseNote());
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
