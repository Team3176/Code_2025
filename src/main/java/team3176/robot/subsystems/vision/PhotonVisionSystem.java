package team3176.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSystem extends SubsystemBase {
  // currently using an body frame that is at the center of the XY of the robot
  // and projected down
  // to the floor Z
  // is an area for further standards with design to pick a body attached frame
  /*
   * ^ +x
   * |
   * +y |
   * <---O +z up
   *
   *
   * cameras have the convention similar reporting position in the camera frame
   * with +x being the axis through the lense and +y to the left, z up
   */

  private static PhotonVisionSystem instance;

  private PhotonVisionSystem() {}

  public static PhotonVisionSystem getInstance() {
    if (PhotonVisionSystem.instance == null) {
      PhotonVisionSystem.instance = new PhotonVisionSystem();
    }

    return PhotonVisionSystem.instance;
  }

  @Override
  public void periodic() {}
}
