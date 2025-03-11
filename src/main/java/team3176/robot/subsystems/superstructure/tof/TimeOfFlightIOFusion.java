package team3176.robot.subsystems.superstructure.tof;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public class TimeOfFlightIOFusion {
  private int id;
  private TimeOfFlight TOF;
  private static TimeOfFlightIOFusion instance;

  public TimeOfFlightIOFusion() {

  }

  private TimeOfFlightIOFusion(int id) {
    this.id = id;
    TOF = new TimeOfFlight(id);
  }
  public int getID(){
    return id;
  }

  public void setViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    TOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

   public double getRangeInches() {
    return Math.round(((getRangeRaw())/(25.4))*100.0)/100.0;
  }

  public double getRangeRaw() {
    return TOF.getRange() - 30.0;
  }

  public double getRangeCentimeters() {
    return getRangeRaw()/10;
  } 

  public void close() {
    TOF.close();
  }

  public void setRange(String sMode, double sampleTime) {
    switch(sMode.toLowerCase()) {
      case "short":
       TOF.setRangingMode(RangingMode.Short, sampleTime);
      case "long":
       TOF.setRangingMode(RangingMode.Long, sampleTime);
      case "medium":
       TOF.setRangingMode(RangingMode.Medium, sampleTime);
    }
  }

    public static TimeOfFlightIOFusion getInstance() {
    if (instance == null) {
      instance = new TimeOfFlightIOFusion();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
