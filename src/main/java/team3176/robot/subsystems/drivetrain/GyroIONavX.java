package team3176.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.SPI;
import team3176.robot.constants.DriveConstants;

import java.util.OptionalDouble;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
  AHRS navX;
  //private final Pigeon2 pigeon = new Pigeon2(5, "canivore");
  //private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  //private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();


  public GyroIONavX() {
    navX = new AHRS(SPI.Port.kMXP);

    /* 
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    tryUntilOk(5, () -> pigeon.setYaw(0.0, 0.25));
    */

    yawTimestampQueue = TalonOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        TalonOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = navX.isConnected();
                  if (valid) {
                    return OptionalDouble.of(navX.getRotation2d().getDegrees());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    //yawPositionQueue = TalonOdometryThread.getInstance().registerSignal(pigeon.getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = navX.isConnected();
    inputs.pitch = navX.getPitch();
    inputs.roll = navX.getRoll();
    inputs.yaw = navX.getYaw();
    inputs.rotation2d = navX.getRotation2d();
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    /* 
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    */

  }

  @Override
  public void reset() {
    navX.reset();
  }
}
