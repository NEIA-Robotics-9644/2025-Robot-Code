package frc.robot.subsystems.intake.sensor;

public interface CoralSensorIO {

  default public boolean coralDetected(){return true;}

  default public void setDisplayLight(boolean on){}
}
