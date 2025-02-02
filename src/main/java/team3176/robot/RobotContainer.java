// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import team3176.robot.commands.WheelRadiusCharacterization;
import team3176.robot.commands.WheelRadiusCharacterization.Direction;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.Visualization;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.leds.LEDS;
import team3176.robot.subsystems.leds.LEDSubsystem;
import team3176.robot.subsystems.superstructure.*;
import team3176.robot.subsystems.vision.PhotonVisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private PowerDistribution pdh;

  // is this why we don't have a compressor? private final Compressor m_Compressor
  private Controller controller;
  private Drivetrain drivetrain;
  private LEDSubsystem leds;
  private Superstructure superstructure;
  private PhotonVisionSystem vision;
  private Visualization visualization;
  private LoggedDashboardChooser<Command> autonChooser;
  private Command choosenAutonomousCommand = new WaitCommand(1.0);
  private Alliance currentAlliance = Alliance.Blue;
  private Trigger endMatchAlert = new Trigger(() -> DriverStation.getMatchTime() < 20);
  private Trigger shooterOverride;
  private Trigger ampOverride;
  //  private Trigger intakeOverride;
  private Trigger visionOverride;
  private LEDS ledsRio;
  private boolean PathPlannerPresent;

  enum PDType {
    CTRE,
    REV
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure what objects will be contstructed
    // Controller controller = null;
    // Drivetrain drivetrain = null;
    // LEDSubsystem leds = null;
    // LEDSubsystem ledsRio = null;
    Superstructure superstructure = null;
    // PhotonVisionSystem vision = null;
    // Visualization visualization = null;
    PDType pdType = PDType.REV; // Should be PdType.REV or PDType.CTRE
    PathPlannerPresent = true; // Should be true or false

    // Construct objects
    if (controller == null) {
      controller = Controller.getInstance();
    }

    if (drivetrain == null) {
      drivetrain = Drivetrain.getInstance();
    }

    if (leds == null) {
      leds = LEDSubsystem.getInstance();
      endMatchAlert.onTrue(leds.EndgameStart());
    }

    if (ledsRio == null) {
      ledsRio = LEDS.getInstance();
    }

    visualization = new Visualization();
    if (Constants.VISION_CONNECTED) {
      vision = PhotonVisionSystem.getInstance();
    }

    if (superstructure == null) {
      superstructure = Superstructure.getInstance();
    }

    if (pdType == PDType.CTRE) {
      pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kCTRE);
    } else pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);

    if (drivetrain != null) {
      drivetrain.setDefaultCommand(
          drivetrain
              .swerveDriveJoysticks(
                  () -> controller.getForward(),
                  () -> controller.getStrafe(),
                  () -> controller.getSpin())
              .withName("default drive"));
    }

    if (leds != null) {
      leds.setDefaultCommand(leds.DefaultLED());
    }

    if (PathPlannerPresent == true) {
      // These all need to be sped up
      NamedCommands.registerCommand("shoot", new WaitCommand(1.0));
      // NamedCommands.registerCommand(    //     "shoot",
      //     superstructure
      //         .aimClose
      //         .alongWith(new WaitCommand(0.5).andThen(superstructure.shoot().withTimeout(0.3)))
      //         .withTimeout(0.8)
      //         .withName("shooting"));

      autonChooser = new LoggedDashboardChooser<>("autonChoice", AutoBuilder.buildAutoChooser());

      SmartDashboard.putData("Auton Choice", autonChooser.getSendableChooser());
    }

    if (controller != null) {
      configureBindings();
    }
  }

  private void configureBindings() {
    /*
     * overrides
     */
    shooterOverride = controller.switchBox.button(1);
    ampOverride = controller.switchBox.button(2);
    // intakeOverride = controller.switchBox.button(3);
    visionOverride = controller.switchBox.button(4);
    /*
     * Translation Stick
     */
    /*     controller
    .transStick
    .button(1)
    .whileTrue(new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE)); */
    controller
        .transStick
        .button(1)
        .whileTrue(
            drivetrain
                .swerveDriveJoysticks(
                    () -> controller.getForward(),
                    () -> controller.getStrafe(),
                    () -> controller.getSpin() * 1.5)
                .withName("boost drive"));

    /*
        controller
        .transStick
        .button(4)
        .whileTrue(
            drivetrain
                .chaseNoteTeleo(
                    () -> controller.getForward(),
                    () -> controller.getStrafe(),
                    () -> controller.getSpin())
                .alongWith(superstructure.intakeNote()));
    */
    controller.transStick.button(5).onTrue(drivetrain.resetPoseToVisionCommand());
    controller
        .transStick
        .button(10)
        .whileTrue(drivetrain.swerveDefenseCommand().withName("swerveDefense"));

    /*
     *  Rotation Stick
     */
    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    /*
     * Operator
     */
    /*
    controller
        .operator
        .rightBumper()
        .whileTrue(superstructure.moveClimbRightPosition(() -> controller.operator.getRightY()))
        .onFalse(superstructure.stopClimbRight());
        */
    // controller.operator.povDown().onTrue(superstructure.intakeNote());

    controller
        .switchBox
        .button(5)
        .whileTrue(new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE));

    controller
        .switchBox
        .button(4)
        .onTrue(drivetrain.setVisionOverride(true))
        .onFalse(drivetrain.setVisionOverride(false));
  }

  public void clearCanFaults() {
    pdh.clearStickyFaults();
  }

  public void printCanFaults() {
    pdh.getStickyFaults();
  }

  public void checkAutonomousSelection(Boolean force) {
    if (autonChooser.get() != null
        && (!choosenAutonomousCommand.equals(autonChooser.get()) || force)) {
      Long start = System.nanoTime();
      choosenAutonomousCommand = autonChooser.get();
      try {
        choosenAutonomousCommand = autonChooser.get();
      } catch (Exception e) {
        System.out.println("[ERROR] could not find" + autonChooser.get().getName());
        System.out.println(e.toString());
      }

      Long totalTime = System.nanoTime() - start;
      System.out.println(
          "Autonomous Selected: ["
              + autonChooser.get().getName()
              + "] generated in "
              + (totalTime / 1000000.0)
              + "ms");
    }
  }

  public void checkAutonomousSelection() {
    checkAutonomousSelection(false);
  }

  public void checkAllaince() {
    // TODO: check the optional return instead of just .get()
    if (DriverStation.getAlliance().orElse(Alliance.Blue) != currentAlliance) {
      currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      // Updated any things that need to change
      System.out.println("changed alliance");
      checkAutonomousSelection(true);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.get();
  }

  public boolean isPathPlannerPresent() {
    boolean isPathPlannerPresent = PathPlannerPresent ? true : false;
    return isPathPlannerPresent;
  }
}
