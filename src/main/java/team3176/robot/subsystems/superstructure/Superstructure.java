package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class Superstructure {
  private static Superstructure instance;

  public Superstructure() {}

  /*
  public Command climbDown() {
    return climb.moveLeftRightPosition(0, 0);
  }
  */

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
