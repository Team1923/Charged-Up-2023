package frc.robot.interfaces;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.interfaces.ColorSensorInterface.GamePiece;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.GamePieceMode;

public class LEDInterface{

	private static LEDInterface ledInterface;

	private DigitalOutput bit1 = new DigitalOutput(4);
	private DigitalOutput bit2 = new DigitalOutput(5);
	private DigitalOutput bit3 = new DigitalOutput(6);


	StateHandler stateHandler = StateHandler.getInstance();

	public static synchronized LEDInterface getInstance() {
		if (ledInterface == null) {
		  ledInterface= new LEDInterface();
		}
		return ledInterface;
	  }

	  public LEDInterface(){
	  }

	  public void updateLed(){
		if(DriverStation.isDisabled()){
			if(!stateHandler.getIsArmGood() && !stateHandler.getIsIntakeGood()){
				bit1.set(false);
				bit2.set(false);
				bit3.set(false);
			}
			else if(stateHandler.getIsArmGood() && !stateHandler.getIsIntakeGood()){
				bit1.set(true);
				bit2.set(false);
				bit3.set(false);
			}
			else if(!stateHandler.getIsArmGood() && stateHandler.getIsIntakeGood()){
				bit1.set(false);
				bit2.set(true);
				bit3.set(false);
			}
			else{
				bit1.set(true);
				bit2.set(true);
				bit3.set(false);
			}
		}
		if(stateHandler.getGamePieceMode() == GamePieceMode.CUBE){
			bit1.set(false);
			bit2.set(false);
			bit3.set(true);
		}
		else{
			bit1.set(true);
			bit2.set(false);
			bit3.set(true);
		}
	  }

}