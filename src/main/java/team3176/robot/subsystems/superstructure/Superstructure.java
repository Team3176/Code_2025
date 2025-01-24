package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;

  public Superstructure() {
    climb = Climb.getInstance();
  }

  /*
  public Command climbDown() {
    return climb.moveLeftRightPosition(0, 0);
  }
  */
  public Command moveClimbLeftPosition(DoubleSupplier position) {
    return climb.moveLeftPosition(position);
  }

  public Command stopClimbLeft() {
    return climb.stopLeft();
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
