package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Elevator.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.List;
import java.util.ArrayList;

// import com.google.gson.Gson;
// import com.google.gson.GsonBuilder;
// import com.google.gson.JsonArray;
// import com.google.gson.JsonElement;
// import com.google.gson.JsonObject;
// import com.google.gson.JsonParser;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Mat;



public class Robot extends TimedRobot {
	private DriveHardware hardware;
	private TorDrive drive;
	private Joystick player1;
	private boolean test;
	private Elevator elevator;
	private statusLights statusLights;
	private DigitalOutput light1;
	private DigitalOutput light2;
	private DigitalOutput light3;
	private boolean starting = true;
	
	// camera
	private static String configFile = "/boot/frc.json";

	public static int team;
	  public static boolean server;
	// public static class CameraConfig {
	// 	public String name;
	// 	public String path;
	// 	public JsonObject config;
	// 	public JsonElement streamConfig;
	//   }
	// public static List<CameraConfig> cameraConfigs = new ArrayList<>();
	public static List<VideoSource> cameras = new ArrayList<>();

	
	public Robot() {
		test = false;
		hardware = new DriveHardware();																																																																																																					
		player1 = new Joystick(0);
		drive = new TorDrive(hardware, player1);

		light1 = new DigitalOutput(23);
        light2 = new DigitalOutput(24);
		light3 = new DigitalOutput(25);
		
		statusLights = new statusLights(light1, light2, light3);
		statusLights.displayRedLights();
		elevator = new Elevator(player1, drive, statusLights);

		
		

//---------------------------- 
/*
		NetworkTableInstance raspberryPi = NetworkTableInstance.getDefault();
		if (!readConfig()) {
			return;
		  }
		  System.out.println("Setting up NetworkTables client for team " + 1198);
		  raspberryPi.startClientTeam(1198);
		
		  for (CameraConfig config : cameraConfigs) {
			cameras.add(startCamera(config));
		  }
*/
// ------------------------
	
		// UsbCamera hatchSideCam = CameraServer.getInstance().startAutomaticCapture(0);
		
		// // hatchSideCam.setExposureManual();
		// CvSink cvsink1 = new CvSink("Hatch Side Cam");
		// hatchSideCam.setBrightness(1);

		// hatchSideCam.setExposureManual(2);
		// // hatchSideCam.setWhiteBalanceAuto();
		// hatchSideCam.setResolution(420, 240);
		// hatchSideCam.setFPS(10);
		// cvsink1.setSource(hatchSideCam);
		// cvsink1.setEnabled(true);

		// UsbCamera cargoSideCam = CameraServer.getInstance().startAutomaticCapture(1);
	

		// CvSink cvsink2 = new CvSink("Cargo Side Cam");
		// cvsink2.setSource(cargoSideCam);
		// 	cargoSideCam.setBrightness(1);
		// 	cargoSideCam.setExposureManual(1);

		// // cargoSideCam.setWhiteBalanceAuto();
		// cargoSideCam.setResolution(420, 240);
		// cargoSideCam.setFPS(10);
		// cvsink2.setEnabled(true);
	}
	@Override
	public void robotInit() {
		starting = true;
		hardware.init();
	}
	@Override
	public void autonomousInit() {
		starting = true;
		if(starting) {
			elevator.init();
			starting = false;
		}
	}
	@Override
	public void autonomousPeriodic() {
		if(starting) {
			elevator.init();
			starting = false;
		}
		drive.Run(test, true);//IT IS NOW TELEOP IN AUTO
		elevator.update();
	}
	@Override
	public void teleopPeriodic() {
		if(starting) {
			elevator.init();
			starting = false;
		}
		elevator.update();
		if(!elevator.climbing()) {
			drive.Run(test, true);//IT IS TELEOP
		}
	}
	@Override
	public void testPeriodic() {
		if(!elevator.climbing()) {
			drive.Run(true, false);//whether or not it is teleop in test mode does not matter
		}
		elevator.update();
	}


// Camera config methods
// ------------------------------------------------------------------------------------------------
// 	public static boolean readCameraConfig(JsonObject config) {
// 		CameraConfig cam = new CameraConfig();
	
// 		// name
// 		JsonElement nameElement = config.get("name");
// 		if (nameElement == null) {
// 		  parseError("could not read camera name");
// 		  return false;
// 		}
// 		cam.name = nameElement.getAsString();
	
// 		// path
// 		JsonElement pathElement = config.get("path");
// 		if (pathElement == null) {
// 		  parseError("camera '" + cam.name + "': could not read path");
// 		  return false;
// 		}
// 		cam.path = pathElement.getAsString();
	
// 		// stream properties
// 		cam.streamConfig = config.get("stream");
	
// 		cam.config = config;
	
// 		cameraConfigs.add(cam);
// 		return true;
// 	  }


// 	public static VideoSource startCamera(CameraConfig config) {
// 		System.out.println("Starting camera '" + config.name + "' on " + config.path);
// 		CameraServer inst = CameraServer.getInstance();
// 		UsbCamera camera = new UsbCamera(config.name, config.path);
// 		MjpegServer server = inst.startAutomaticCapture(camera);
	
// 		Gson gson = new GsonBuilder().create();
	
// 		camera.setConfigJson(gson.toJson(config.config));
// 		camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
	
// 		if (config.streamConfig != null) {
// 		  server.setConfigJson(gson.toJson(config.streamConfig));
// 		}
	
// 		return camera;
// 	  }


//  public static boolean readConfig() {
//     // parse file
//     JsonElement top;
//     try {
//       top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
//     } catch (IOException ex) {
//       System.err.println("could not open '" + configFile + "': " + ex);
//       return false;
//     }

//     // top level must be an object
//     if (!top.isJsonObject()) {
//       parseError("must be JSON object");
//       return false;
//     }
//     JsonObject obj = top.getAsJsonObject();

//     // team number
//     JsonElement teamElement = obj.get("team");
//     if (teamElement == null) {
//       parseError("could not read team number");
//       return false;
//     }
//     team = teamElement.getAsInt();

//     // ntmode (optional)
//     if (obj.has("ntmode")) {
//       String str = obj.get("ntmode").getAsString();
//       if ("client".equalsIgnoreCase(str)) {
//         server = false;
//       } else if ("server".equalsIgnoreCase(str)) {
//         server = true;
//       } else {
//         parseError("could not understand ntmode value '" + str + "'");
//       }
//     }

//     // cameras
//     JsonElement camerasElement = obj.get("cameras");
//     if (camerasElement == null) {
//       parseError("could not read cameras");
//       return false;
//     }
//     JsonArray cameras = camerasElement.getAsJsonArray();
//     for (JsonElement camera : cameras) {
//       if (!readCameraConfig(camera.getAsJsonObject())) {
//         return false;
//       }
//     }

//     if (obj.has("switched cameras")) {
//       JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
//       for (JsonElement camera : switchedCameras) {
//         if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
//           return false;
//         }
//       }
//     }

//     return true;
//   }


	/*
	 *  The following are a bunch of accessor methods to obtain input from the controller.
	 */
	public double getLeftX(){
		return player1.getRawAxis(0);
	}

	public double getLeftY(){
		return player1.getRawAxis(1);
	}

	public double getRightX(){
		return player1.getRawAxis(4);
	}

	public double getRightTrigger(){
		return player1.getRawAxis(3);
	}

	public boolean getShiftButton(){
		return player1.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player1.getRawButton(6);
	}

	public boolean getButtonA(){
		return player1.getRawButton(1);
	}

	public boolean getButtonB(){
		return player1.getRawButton(2);
	}

	public boolean getButtonX(){
		return player1.getRawButton(3);
	}

	public boolean getButtonY(){
		return player1.getRawButton(4);
	}
}
