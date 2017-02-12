//climber is 7 and 6

//Drive Train pwm ports: fl 0 ; fr 1; br 2; bl 4;
package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.*;


import com.kauailabs.navx_mxp.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	
	public enum AutoState {
		START, GO, TURN, ALIGN, PLACE, FINISH
	}
	public enum TurnDirection{
		LEFT, RIGHT, STRAIGHT
	}
	AutoState CurrentAutoState = AutoState.START;
	
	TurnDirection Starter = TurnDirection.RIGHT;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	//DRIVE TRAIN SPEED CONTROLLERS
	VictorSP fL;
	VictorSP fR;
	VictorSP bL;
	VictorSP bR;
	
	//DRIVE TRAIN ENCODERS
	Encoder leftEncoder, rightEncoder;
	int leftCount, rightCount;
	double driveDistance, dpp;
	
	double offsetFactor;
	
	//DRIVE JOYS
	Joystick joy;
	Joystick haroldsjoystick;
	DigitalInput lim;
	RobotDrive drive;
	VictorSP leadScrew;
	
	//MECHANISMS
	DoubleSolenoid gearPiss;
	DoubleSolenoid lidPiss;
	
	CameraServer server;
	
	//VISION
	NetworkTable table;
	boolean found;
	String offset;
	double distance;
	
	boolean straight;
	
	//AUTON
	Timer timer;
	double autoLeft, autoRight;
	int correctingCheck;

	float drivePower;
	
	double testTime = 1;
	
	SerialPort serial_port;
	// IMU imu; // This class can be used w/nav6 and navX MXP.
	// IMUAdvanced imu; // This class can be used w/nav6 and navX MXP.
	AHRS imu; // This class can only be used w/the navX MXP.


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		fL = new VictorSP(0);
		bL = new VictorSP(3);
		fR = new VictorSP(1);
		bR = new VictorSP(2);
		joy = new Joystick(0);
		
		offsetFactor = 0.91; 
		straight = true;
		// haroldsjoystick = new Joystick(1);
		// haroldsmotor = new VictorSP(4);
		drive = new RobotDrive(fL, bL, fR, bR);
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivePower = 0.5f;
		lidPiss = new DoubleSolenoid(4, 5);
		gearPiss = new DoubleSolenoid(2, 3);
		server = CameraServer.getInstance();
		
		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		
		table = NetworkTable.getTable("VisionTable");
		
		try {

			// Use SerialPort.Port.kOnboard if connecting nav6 to Roborio Rs-232
			// port
			// Use SerialPort.Port.kMXP if connecting navX MXP to the RoboRio
			// MXP port
			// Use SerialPort.Port.kUSB if connecting nav6 or navX MXP to the
			// RoboRio USB port

			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);

			// You can add a second parameter to modify the
			// update rate (in hz) from. The minimum is 4.
			// The maximum (and the default) is 100 on a nav6, 60 on a navX MXP.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.
			// The recommended maximum update rate is 50Hz

			// You can also use the IMUAdvanced class for advanced
			// features on a nav6 or a navX MXP.

			// You can also use the AHRS class for advanced features on
			// a navX MXP. This offers superior performance to the
			// IMU Advanced class, and also access to 9-axis headings
			// and magnetic disturbance detection. This class also offers
			// access to altitude/barometric pressure data from a
			// navX MXP Aero.

			byte update_rate_hz = 50;
			// imu = new IMU(serial_port,update_rate_hz);
			// imu = new IMUAdvanced(serial_port,update_rate_hz);
			imu = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {

		}

		

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
		
		resetEncoders();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	//Overestimating distance by 2 inches
	@Override
	public void autonomousPeriodic() {
		/*Scheduler.getInstance().run();
		// if(swit.get()){

		
		 * } else{ motor.set(0); }
		 
		
		found = table.getBoolean("found", false);
		offset = table.getString("offset", "default");
		distance = table.getNumber("distance", 100.0);
		
		getEncoders();
		
		switch(CurrentAutoState) {
		case START:
			//Currently going to TURN because we haven't calibrated encoders for GO
			CurrentAutoState = AutoState.TURN;
			break;
		case GO:

			if(driveDistance <= 100){
				drive.tankDrive(0.8,0.8);	
			} else {
				drive.tankDrive(0.0, 0.0);
				CurrentAutoState = AutoState.TURN;
			}
			break;
		case TURN:
			
			//found is true when the contours are within width height ratios and alignment specifications
			if(found){
				System.out.println("Tape Found? " + found);
				autoLeft = 0.0;
				autoRight = 0.0;
				CurrentAutoState = AutoState.ALIGN;
			} else {
				if(Starter == TurnDirection.RIGHT){
				autoLeft = 0.4;
				autoRight = -0.4;
				}
				else if(Starter == TurnDirection.LEFT){
					autoLeft = -0.4;
					autoRight = 0.4;
				}
				else{
					autoLeft = 0;
					autoRight = 0;
				}
			}
			break;
		case ALIGN:
			System.out.println(offset);
			if(found){
				System.out.println("Tape Found? " + found);
				if(offset.equals("left")){
					System.out.println("Offset: " + offset);
					autoLeft = -0.4;
					autoRight = 0.4;
					
				} else if(offset.equals("right")){
					System.out.println("Offset: " + offset);
					autoRight = -0.4;
					autoLeft = 0.4;
				} else if(offset.equals("centered")){
					autoRight = 0.0;
					autoLeft = 0.0;
					CurrentAutoState = AutoState.PLACE;
				} System.out.println(autoLeft); System.out.println(autoRight);
			} else {
				if(i == 1 && autoRight != 0.0){
					autoLeft = 0.4;
					autoRight = 0.0;
					i++;
				}else if(i==1 && autoLeft != 0.0){
					autoLeft = 0.0;
					autoRight = 0.4;
					i++;
				}
				CurrentAutoState = AutoState.TURN;
				}
				break;
		case PLACE:
			System.out.println(offset);
			if(offset.equals("centered")){
				if(34 <= distance && distance<= 38){
					autoLeft = 0.0;
					autoRight = 0.0;
					lidPiss.set(DoubleSolenoid.Value.kForward);
					CurrentAutoState = AutoState.FINISH;
				} else if (distance > 34.0){
					autoLeft = 0.4;
					autoRight = 0.4;
				}else {
					autoLeft = 0;
					autoRight = 0;
				}
			}
			else{
				CurrentAutoState = AutoState.ALIGN;
			}
			break;
		case FINISH:
			lidPiss.set(DoubleSolenoid.Value.kOff);
			break;
			}
		drive.tankDrive(autoLeft, autoRight);
		System.out.println("Distance Away: " + distance);
		System.out.println("Current Autonomous State: " + CurrentAutoState);*/
		
		drive.tankDrive(0.7*offsetFactor, 0.7);
	
		
	}
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		if(joy.getRawButton(5)){
			drivePower = 1;
		}
		else{
			drivePower=.8f;
		}
		Scheduler.getInstance().run();
		drive.tankDrive((-joy.getRawAxis(1) * offsetFactor) * drivePower, -joy.getRawAxis(5) * drivePower);
		
		
		//Straight driving
		if (joy.getRawButton(6)) {
			drive.tankDrive((-joy.getRawAxis(1) *offsetFactor) * drivePower, -joy.getRawAxis(1) * drivePower);
		}
		
		//Backwards driving
		if (joy.getRawButton(3)) {
			drive.tankDrive((joy.getRawAxis(1) * offsetFactor) * drivePower, joy.getRawAxis(5) * drivePower);
			
			//Backwards and Straight
			if (joy.getRawButton(6)) {
				drive.tankDrive((joy.getRawAxis(1) * offsetFactor) * drivePower, joy.getRawAxis(1) * drivePower);
			}
		}
		
		//Pneumatics actuating
		if (joy.getRawButton(1)) {
			lidPiss.set(DoubleSolenoid.Value.kForward);
		} else if (joy.getRawButton(2)) {
			lidPiss.set(DoubleSolenoid.Value.kReverse);
		} else {
			lidPiss.set(DoubleSolenoid.Value.kOff);
		}
		if (joy.getRawButton(3)) {
			gearPiss.set(DoubleSolenoid.Value.kForward);
		} else if (joy.getRawButton(4)) {
			gearPiss.set(DoubleSolenoid.Value.kReverse);
		}
	
		
	//	gearPiss.set(DoubleSolenoid.Value.kForward);
	
		found = table.getBoolean("found", false);
		offset = table.getString("centered", "default");
		distance = table.getNumber("distance", 100.0);
		

		testTime++;
		table.putNumber("time", testTime);
		System.out.println(testTime);
		System.out.println("Found?" + found + "offset?" + offset + "distance" + distance);
		
		//System.out.println("Gyro: " + imu.getYaw());
		System.out.println("Left Encoder:" + leftEncoder.get() + "Right Encoder: "+rightEncoder.get());
		System.out.println("Left Joy: " + joy.getRawAxis(1) + "Right Joy: " + joy.getRawAxis(5));
		if(joy.getRawButton(9)){
			resetEncoders();
		} if(joy.getRawButton(10)){
			imu.zeroYaw();
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	public void getEncoders(){
		leftCount = leftEncoder.get();
		rightCount = rightEncoder.get();
		
		driveDistance = dpp * ((leftCount + rightCount)/2);
		
	}
	public void resetEncoders(){
		leftCount = 0;
		rightCount = 0;
		driveDistance = 0;
		
		leftEncoder.reset();
		rightEncoder.reset();
		
		
	}
}