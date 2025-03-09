package team3176.robot.subsystems.superstructure.tof;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.superstructure.elevator.Elevator;
import team3176.robot.subsystems.superstructure.elevator.ElevatorIOTalon;

public class timeofflight {
    private static TimeOfFlight instance;

    

    private TimeOfFlight m_tof = new TimeOfFlight(1);
    public double TOFTest () {
         return m_tof.getRange();
    }
    
    public Command tofdistance() {
        return this.runEnd(
            () -> {
              
            },
            () -> io.setLeftElevatorH(0.0));
        }
    public static TimeOfFlight getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new TimeOfFlight(Hardwaremap.tof_id);
        System.out.println("Tof instance created for Mode.REAL");
      } else {
        System.out.println("Tof instance created for Mode.SIM");
      }
    }
    return instance;
  }
    
}

    


