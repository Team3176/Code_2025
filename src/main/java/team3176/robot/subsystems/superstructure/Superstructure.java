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


  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
