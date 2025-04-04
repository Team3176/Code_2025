// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more detail.

package com.team3176.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.team3176.robot.generated.TunerConstants;
import com.team3176.robot.commands.*;
import com.team3176.robot.commands.AlignToReef.FieldBranchSide;
//import com.team3176.robot.commands.AlignReef.TargetLoc; // for enum TargetLoc
import com.team3176.robot.subsystems.controller.Controller;
import com.team3176.robot.subsystems.drivetrain.Drive;
import com.team3176.robot.subsystems.drivetrain.GyroIOPigeon2;
import com.team3176.robot.subsystems.drivetrain.ModuleIOTalonFX;
import com.team3176.robot.subsystems.superstructure.Superstructure;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.vision.Vision;
import com.team3176.robot.subsystems.vision.VisionIO;
import com.team3176.robot.subsystems.vision.VisionIOPhotonVision;
import static com.team3176.robot.subsystems.vision.VisionConstants.*;
//import com.team3176.robot.subsystems.tof.TimeOfFlightSystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive = Drive.getInstance();

  //private final Vision vision = new Vision(); //  Vision.getInstance();
  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final Controller controller = Controller.getInstance();

  // Superstructure
  private final Superstructure superstructure = Superstructure.getInstance();
  //private final TimeOfFlightSystem tofSystem = TimeOfFlightSystem.getInstance(); // TOF system
  private final Vision vision;

  private Alliance currentAlliance = Alliance.Blue;
//  private Trigger endMatchAlert = new Trigger(() -> DriverStation.getMatchtime() < 20 );
  private Trigger visionOverride; 
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private AlignToReef alignmentCommandFactory = null;
  private VariableAutos variableAutoFactory = null;
  public final DynamicsCommandFactory dynamics = null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public PathPlannerAuto autocommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    vision = new Vision(drive::addVisionMeasurement,
        new VisionIOPhotonVision(camera1Name, robotToCamera1),
        new VisionIOPhotonVision(camera2Name, robotToCamera2),
       new VisionIOPhotonVision(camera3Name, robotToCamera3),
        new VisionIOPhotonVision(camera4Name, robotToCamera4));
    // switch (Constants.currentMode) {
    // case REAL:
    // Real robot, instantiate hardware IO implementations
    //drive =
    //    new Drive(
    //        new GyroIOPigeon2(),
    //        new ModuleIOTalonFX(TunerConstants.FrontLeft),
    //        new ModuleIOTalonFX(TunerConstants.FrontRight),
    //        new ModuleIOTalonFX(TunerConstants.BackLeft),
    //        new ModuleIOTalonFX(TunerConstants.BackRight));
    // break;

    /*    case SIM:
      // Sim robot, instantiate physics sim IO implementations
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      break;

    default:
      // Replayed robot, disable IO implementations
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      break;
      */
    // }
    alignmentCommandFactory = new AlignToReef(drive, fieldLayout);
    variableAutoFactory = new VariableAutos(alignmentCommandFactory, dynamics, drive);


    NamedCommands.registerCommand("L2", superstructure.goToL2()
        .withDeadline(new WaitCommand(1.5).andThen(superstructure.shoot().withTimeout(1)))
        .andThen(superstructure.stopRollers())
        .andThen(superstructure.goToL0().withTimeout(1)));
    NamedCommands.registerCommand("L3", superstructure.goToL3()
        .withDeadline(new WaitCommand(1.5).andThen(superstructure.shoot().withTimeout(1)))
        .andThen(superstructure.stopRollers())
        .andThen(superstructure.goToL0().withTimeout(1)));
    // this keeps the elevator up while we shoot and then brings it down
    NamedCommands.registerCommand("L4", superstructure.goToL4()
        .withDeadline(new WaitCommand(1.5).andThen(superstructure.shoot().withTimeout(1)))
        .andThen(superstructure.stopRollers())
        .andThen(superstructure.goToL0().withTimeout(1.5)));

    NamedCommands.registerCommand("DeAlgae", superstructure.goToA3()
        .withTimeout(1.5).andThen(superstructure.deAlgae().withTimeout(1.5))
        .andThen(superstructure.goToA2())
        .andThen(superstructure.deAlgaeHome()));
    NamedCommands.registerCommand("IntakeAlgaeHigh", superstructure.goToA2().withDeadline(new WaitCommand(3)));
    NamedCommands.registerCommand("IntakeAlgaeLow", superstructure.goToA1().withDeadline(new WaitCommand(3)));
    NamedCommands.registerCommand("AlgaeSqueeze", superstructure.algaeSqueeze().withDeadline(new WaitCommand(1.5)));
    NamedCommands.registerCommand("Net", superstructure.goToL4()
        .withDeadline(new WaitCommand(1.5).andThen(superstructure.shootAlgae().withTimeout(1)))
        .andThen(superstructure.stopRollers())
        .andThen(superstructure.goToL0().withTimeout(1.2)));

    //NamedCommands.registerCommand("L4Auto", superstructure.goToL4().andThen());

/*     NamedCommands.registerCommand("L1", superstructure.goToL1().alongWith(Arm.getInstance().setPivotCurrents())
        .withDeadline(new WaitCommand(2.5).andThen(superstructure.deAlgaePositive().withTimeout(2)))
        .andThen(superstructure.deAlgaeNegative())); 
 */
    NamedCommands.registerCommand("intake", superstructure.runRollersIn().withTimeout(2));
    //autocommand = new PathPlannerAuto("2c0aCcG4IfoH4");
    //autocommand.event("intake").onTrue(superstructure.runRollersIn());
    //new EventTrigger("intake").onTrue(superstructure.runRollersIn().withTimeout(1));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption
    (
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    ///// SETUP OVERRIDE BOX ////////
    visionOverride = controller.switchBox.button(4);




    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getForward(),
            () -> controller.getStrafe(),
            () -> controller.getSpin()
        )
    );
    
    // Lock to 0° when A button is held
    /*controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));
    */

    //controller.transStick.pov(270).onTrue(new PrintCommand("Aligning Left").andThen(new AlignReef(TargetLoc.LEFT).andThen(new PrintCommand("Aligning Left"))));
    //controller.transStick.pov(0).onTrue(new AlignReef(TargetLoc.CENTER));
    //controller.transStick.pov(90).onTrue(new AlignReef(TargetLoc.RIGHT));

    // Switch to X pattern when X button is pressed
    controller.transStick.button(4).whileTrue(Commands.runOnce(drive::stopWithX, drive));

    //BOOST ME BABY *2
    controller.rotStick.button(1).
        whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> controller.getForward() *2,
                () -> controller.getStrafe() *2,
                () -> controller.getSpin() *2
            )
        );
    
    // Shoot
    controller.transStick.button(1).onTrue(superstructure.shoot()).onFalse(superstructure.stopRollers());
    controller.transStick.button(2).onTrue(superstructure.shootAlgae()).onFalse(superstructure.stopRollers());
   
    //transStick Stow Elevator
    controller.transStick.button(11).onTrue(superstructure.goToL0());   

    //transStick Climb 
    controller.transStick.button(16).and(controller.transStick.button(15)).whileTrue(superstructure.transStickClimbExtend());
    controller.transStick.button(16).and(controller.transStick.button(14)).whileTrue(superstructure.transStickClimbRetract());
   
   
   
    //RobotCentric ME BABY 
    controller.rotStick.button(2).
        whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getForward(),
                () -> -controller.getStrafe(),
                () -> controller.getSpin(),
                () -> true
            )
        );

    controller.rotStick.button(3).whileTrue(
            alignmentCommandFactory.generateCommand(FieldBranchSide.LEFT)//.finallyDo((boolean interrupted) -> {
            //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
            // })
                .withName("Align Left Branch")
            );
    
    controller.rotStick.button(4).whileTrue(
            alignmentCommandFactory.generateCommand(FieldBranchSide.RIGHT)//.finallyDo((boolean interrupted) -> {
            //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
            // })
            .withName("Align Right Branch")
            );
     controller.rotStick.pov(0).whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> 0.25,
            () -> 0.0,
            () -> 0.0,
            () -> true
        )
    );
        
    controller.rotStick.pov(180).whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> -0.25,
            () -> 0.0,
            () -> 0.0,
            () -> true
         )
    );
        
    controller.rotStick.pov(270).whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> -0.25,
            () -> 0.0,
            () -> 0.0,
            () -> true)
            //.until(() -> tofSystem.isAlignedCenter()
            //.andThen(() -> drive.stop())
            
         )
    ;
        

    controller.rotStick.pov(90).whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> 0.25,
            () -> 0.0,
            () -> 0.0,
            () -> true)
            //.until(() -> tofSystem.isAlignedLeft()
            //.andThen(() -> drive.stop())
         )
    ;
       
    

        //controller.operator.back().and(controller.rotStick.button(12)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //controller.operator.back().and(controller.rotStick.button(13)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //controller.operator.back().and(controller.rotStick.button(14)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //controller.operator.back().and(controller.rotStick.button(15)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));





    
    // Reset gyro to 0° when B button is pressed
    controller
        .rotStick.button(8)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));


    // Shoot
//    controller.transStick.button(1).onTrue(superstructure.shoot()).onFalse(superstructure.stopRollers());


    // ***** OPERATOR CONTROLLER *****
// Climb buttons
    // Max retraction position = -70
    // Staring configuration = 0 to -5
    // Max extension = 
    controller.operator.leftBumper().whileTrue(superstructure.testClimbManual(() -> -controller.operator.getLeftY()));
     
    // Scoring Positions
    controller.operator.a().onTrue(superstructure.goToL1()); //.onFalse(superstructure.goToL0()); 
    controller.operator.x().onTrue(superstructure.goToL2()); //.onFalse(superstructure.goToL0());    
    controller.operator.y().onTrue(superstructure.goToL3()); //.onFalse(superstructure.goToL0());    
    controller.operator.b().onTrue(superstructure.goToL4()); //.onFalse(superstructure.goToL0());   
    //controller.operator.y().onTrue(superstructure.deAlgaePositive());
    //controller.operator.b().onTrue(superstructure.testDeAlgae());

    
    controller.operator.rightTrigger(0.8).and(controller.operator.rightStick().onTrue(superstructure.resetElevatorHome()));
    controller.operator.pov(180).onTrue(superstructure.goToL0()); 
    controller.operator.pov(270)
        .whileTrue(superstructure.goToA1())
        .onFalse(superstructure.algaeSqueeze());//.onTrue(superstructure.goToA1());

   
    controller.operator.pov(0)
        .whileTrue(superstructure.goToA2())
        .onFalse(superstructure.algaeSqueeze());
    controller.operator.pov(90).whileTrue(superstructure.deAlgae()).onFalse(superstructure.deAlgaeHome());//.onTrue(superstructure.goToA3());
    controller.operator.rightStick().and(controller.operator.leftStick()).whileTrue(superstructure.elevatorSetHome()); // Hold both sticks to go to L0

    // Human Load Positions and Rollers
    //controller.operator.rightBumper().onTrue(superstructure.deAlgae()).onFalse(superstructure.algaeToHome()); //.onFalse(superstructure.goToL0());
    //controller.operator.rightBumper().onTrue(superstructure.deAlgaePositive());// .onFalse(superstructure.algaeToHome()); //.onFalse(superstructure.goToL0());
    controller.operator.leftTrigger(0.8).whileTrue(superstructure.runRollersIn()).onFalse(superstructure.stopRollers());
    //controller.operator.start().onTrue(superstructure.algaeToHome());
    controller.operator.start().whileTrue(superstructure.deAlgae()).onFalse(superstructure.deAlgaeHome());
    controller.operator.back().onTrue(superstructure.stopRollers());


    
    
      
    //controller.operator.a().onTrue(superstructure.armVoltPos()).onFalse(superstructure.arm2Home());
    //controller.operator.rightTrigger(.90).whileTrue(superstructure.armVoltPosManual(() -> controller.operator.getRightY()));
    //controller.operator.leftBumper().whileTrue(superstructure.armVoltVelManual(() -> controller.operator.getLeftY())).onFalse(superstructure.stopRollers());
    //controller.operator.b().whileTrue(superstructure.armVoltVel());
    //controller.operator.leftBumper().onTrue(superstructure.testElevator()).onFalse(superstructure.goToL0());
    controller.operator.rightTrigger(.90).whileTrue(superstructure.testElevatorManual(() -> controller.operator.getRightY()));
    
    // Climb buttons
    // Max retraction position = ~+70 // Starting configuration = 0 to -5 // Max extension = ~-150
//    controller.operator.leftBumper().whileTrue(superstructure.testClimbManual(() -> -controller.operator.getLeftY()));
//    controller.transStick.button(16).and(controller.transStick.button(15)).whileTrue(superstructure.transStickClimbExtend());
//    controller.transStick.button(16).and(controller.transStick.button(14)).whileTrue(superstructure.transStickClimbRetract());
     
    // Scoring Positions (States)
//    controller.operator.a().onTrue(superstructure.goToL1()); //.onFalse(superstructure.goToL0()); 
//    controller.operator.x().onTrue(superstructure.goToL2()); //.onFalse(superstructure.goToL0());    
//    controller.operator.y().onTrue(superstructure.goToL3()); //.onFalse(superstructure.goToL0());    
//    controller.operator.b().onTrue(superstructure.goToL4()); //.onFalse(superstructure.goToL0());  
    //controller.operator.pov(270).onTrue(superstructure.goToA1()); 
    //controller.operator.pov(0).onTrue(superstructure.goToA2()); 
    //controller.operator.pov(90).onTrue(superstructure.goToA3()); 
 //   controller.operator.pov(180).onTrue(superstructure.goToL0()); 
 //   controller.transStick.button(11).onTrue(superstructure.goToL0());   
    
    // Scoring Position (Manual)
//    controller.operator.rightTrigger(.90).whileTrue(superstructure.testElevatorManual(() -> controller.operator.getRightY()));
    
    // Human Load Positions and Rollers
    //controller.operator.rightBumper().onTrue(superstructure.goToHumanLoad()); //.onFalse(superstructure.goToL0());
 //   controller.operator.leftTrigger(0.8).whileTrue(superstructure.runRollersIn()).onFalse(superstructure.stopRollers());

    
    
      
    //controller.operator.a().onTrue(superstructure.armVoltPos()).onFalse(superstructure.arm2Home());
    //controller.operator.rightTrigger(.90).whileTrue(superstructure.armVoltPosManual(() -> controller.operator.getRightY()));
    //controller.operator.leftBumper().whileTrue(superstructure.armVoltVelManual(() -> controller.operator.getLeftY())).onFalse(superstructure.stopRollers());
    //controller.operator.b().whileTrue(superstructure.armVoltVel());
    //controller.operator.leftBumper().onTrue(superstructure.testElevator()).onFalse(superstructure.goToL0());


    controller.switchBox.button(4).onTrue(drive.setVisionOverride(true)).onFalse(drive.setVisionOverride(false));

  }


  public void getClimbOutaWay() {
    superstructure.getInstance().getClimbOutaWay();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

   public static AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
}
