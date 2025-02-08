package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
// import java.util.function.IntSupplier;
import java.util.function.DoubleSupplier;
import team3176.robot.subsystems.superstructure.intake.Intake;

public class Superstructure {
  private static Superstructure instance;
  private Intake intake;

  public Superstructure() {
    intake = Intake.getInstance();
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }

  public Command deployIntakePivot() {
    return intake.deployPivot();
  }

  public Command retractIntakePivot() {
    return intake.retractPivot();
  }

  public Command movePivot(DoubleSupplier position) {
    return intake.movePivotPid(position);
  }

  public Command movePivotVelocity(DoubleSupplier velocity) {
    return intake.moveRollerVelocity(velocity);
  }

  public Command setIntakePosition(DoubleSupplier position) {
    return intake.setIntakePosition(position);
  }

  public Command moveIntakePosition(DoubleSupplier position) {
    return intake.moveIntakePosition(position);
  }
}
