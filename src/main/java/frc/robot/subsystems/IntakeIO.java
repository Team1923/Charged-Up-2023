package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/*
    this is essentially an interface that deals with any potential constants that we want to log 
    on the robot. 
    for an intake, i'll just have a constant for speed, position a boolean for if it's running. 
    this is only meant to be a framework to get the constants working. see the subsystem for more info. 
*/
public interface IntakeIO {
    public static class IntakeIOInputs implements LoggableInputs{ //this is how they structure their class naming

        public double speed = 0.0;
        public double position = 0;
        public boolean isRunning = false;

        /*
            the IntakeIOInputs inherits stuff from LoggableInputs. 
            basically we need to include the "toLog" and "fromLog" methods to update our constants and get them
        */

        public void toLog(LogTable table) {
            table.put("Speed", speed);
            table.put("Motor Position", position);
            table.put("Is Running?", isRunning);
            
        }

        public void fromLog(LogTable table) {
            speed = table.getDouble("Speed", speed);
            position = table.getDouble("Motor Position", position);
            isRunning = table.getBoolean("Is Running?", isRunning);
            
        } 
        
    }

    //this allows us to update the inputs
    public default void updateInputs(IntakeIOInputs inputs){} //you can't actually implement a method in an interface
    
}
