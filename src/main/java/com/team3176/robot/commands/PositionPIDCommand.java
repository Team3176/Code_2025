package com.team3176.robot.commands;

import static com.team3176.robot.constants.DriveConstants.AutoConstants.kEndTriggerDebounce;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kPositionTolerance;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kRotationTolerance;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kSpeedTolerance;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.team3176.robot.constants.DriveConstants;
import com.team3176.robot.subsystems.drivetrain.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PositionPIDCommand extends Command{
    
    public Drive mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = DriveConstants.AutoConstants.kAutoAlignPIDController;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final Timer timer = new Timer();

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();



    private PositionPIDCommand(Drive mSwerve, Pose2d goalPose) {
        this.mSwerve = Drive.getInstance();
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

            var rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

            var speed = mSwerve.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

            // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(Drive swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.runVelocity(new ChassisSpeeds(0,0,0));
            swerve.stopWithX();
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.runVelocity(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );

        xErrLogger.accept(mSwerve.getPose().getX() - goalPose.getX());
        yErrLogger.accept(mSwerve.getPose().getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.stop();

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            //+ "\nVelocity value: " + mSwerve.getSpeed() + "m/s"
        );
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
