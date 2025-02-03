package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
//import team3176.robot.subsystems.superstructure.arm.Arm;
//import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.drivetrain.Drivetrain;
//import team3176.robot.subsystems.superstructure.elevator.Elevator;
//import team3176.robot.subsystems.superstructure.intake.Intake;

public class Superstructure {
  private static Superstructure instance;
  //private Arm arm;
  //private Climb climb;
  private Drivetrain drivetrain;
  //private Elevator elevator;
  //private Intake intake;



  public Superstructure() {
    //Arm arm = null;
    //Climb climb = null;
    //Drivetrain drivetrain = null;
    //Elevator elevator = null;
    //Intake intake = null;

    /* 
    if (arm == null) {
      arm = Arm.getInstance();
    }
    */

    /*
    if (climb == null) {
      climb = Climb.getInstance();
    } 
    */

    /* 
    if (drivetrain == null) {
      drivetrain = Drivetrain.getInstance();
    }
    */

    /*
    if (elevator == null) {
      elevator = Elevator.getInstance();
    }
    */

    /*
    if (intake == null) {
      intake = Intake.getInstance();
    }
    */ 

  }

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
