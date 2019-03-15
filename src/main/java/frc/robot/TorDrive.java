package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class TorDrive {//this is the upper layer of the drive
	private DriveController ActiveController;
	// private MotionProfileDriveController MPDC;//Motion Profiling Drive Controller
	private DriveController ADC;//Arcade Drive Controller
	private DriveController TDC;//Test Drive Controller
	private DriveHardware hardware;
	private Joystick player1;
	private double leftOutput;
	private double rightOutput;
	
	private static enum driveStateRunner {
		// MotionProfile,
		ArcadeDrive,
		TestDrive;
		private driveStateRunner() {}
	}
	private driveStateRunner drive = driveStateRunner.TestDrive;
	
	public TorDrive(DriveHardware hardware, Joystick player1) {
		this.player1 = player1;
		this.hardware = hardware;
		ActiveController = new TestDriveController(hardware, player1);//starts out as nothing
		// MPDC = new MotionProfileDriveController(hardware, player1);
		ADC = new ArcadeDriveController(hardware, player1);
		TDC = new TestDriveController(hardware, player1);
	}

	public void setHatchMode(boolean hatchMode) {
		ADC.setHatchMode(hatchMode);
	}
	
	public void Run(boolean testing, boolean isTeleop) {
		if(testing) {
			drive = driveStateRunner.TestDrive;
		} else {
			if(isTeleop) {
				drive = driveStateRunner.ArcadeDrive;
				//handles the shifting
				if(player1.getRawButton(5)) {
					hardware.shiftToLowGear();
				} else {
					hardware.shiftToHighGear();
				}
			} else {
				hardware.shiftToLowGear();//we should first do MP in low gear
				// drive = driveStateRunner.MotionProfile;
			}
		}
		//runs all of them to update their values
		// MPDC.run();
		ADC.run();
		TDC.run();
		
		//sets the activeController to one of them so the active controller will get all the values
		switch(drive) {
		// case MotionProfile:
		// 	ActiveController = MPDC;
		// 	break;
		case ArcadeDrive:
			ActiveController = ADC;
			break;
		case TestDrive:
			ActiveController = TDC;
			break;
		}
		//gets the values from the active controller and sets them to the hardware
		leftOutput = ActiveController.getLeftOutput();
		rightOutput = ActiveController.getRightOutput();
		hardware.setMotorSpeeds(rightOutput, leftOutput);
	}

	public void Run(double speed) {
		hardware.setMotorSpeeds(speed, speed);
	}
	
	//we are only running trajectories in MP so that is why it only takes from the MPDC
	// public boolean MPTrajectoryIsComplete() {
	// 	return MPDC.activeTrajectoryIsComplete();
	// }

	//we are only running trajectories in MP so that is why it only takes from the MPDC
	// public void executeTrajectory(TorTrajectory trajectory, long millisecondsToTimeOut) {
	// 	MPDC.executeTrajectory(trajectory, millisecondsToTimeOut);
	// }
}