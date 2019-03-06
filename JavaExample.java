package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
 
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
 
    /*** Initializes all classes ***/
 
    private boolean driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen;
    private NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw,
            videoTimestamp;
    private XboxController driverJoy;
    private double targetAngle;
    NetworkTableInstance instance;
    NetworkTable chickenVision;
    /*** EDIT TO CURRENT CONFIGURATION ***/
    Spark leftMotor, rightMotor;
    DifferentialDrive drive;
    Hand leftSide, rightSide;
 
    public Robot() {
        super();
 
    }
 
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        instance = NetworkTableInstance.getDefault();
 
        chickenVision = instance.getTable("ChickenVision");
 
        tapeDetected = chickenVision.getEntry("tapeDetected");
        cargoDetected = chickenVision.getEntry("cargoDetected");
        tapeYaw = chickenVision.getEntry("tapeYaw");
        cargoYaw = chickenVision.getEntry("cargoYaw");
 
        driveWanted = chickenVision.getEntry("Driver");
        tapeWanted = chickenVision.getEntry("Tape");
        cargoWanted = chickenVision.getEntry("Cargo");
 
        videoTimestamp = chickenVision.getEntry("VideoTimestamp");
 
        tapeVision = cargoVision = false;
        driverVision = true;
        driverJoy = new XboxController(0);
        /*** EDIT TO CURRENT CONFIGURATION ***/
        leftMotor = new Spark(0);
        rightMotor = new Spark(1);
        drive = new DifferentialDrive(leftMotor, rightMotor);
        
        leftSide = Hand.kLeft;
        rightSide = Hand.kRight;
        targetAngle = 0;
        /*** Update Dashboard ***/
 
    }
 
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    @Override
    public void autonomousInit() {
 
    }
 
    /**
     * This function is called periodically during autonomous, but is mainly used
     * for logging
     */
    @Override
    public void autonomousPeriodic() {
 
    }
 
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    @Override
    public void teleopInit() {
 
    }
 
    /**
     * This function is called periodically during operator control Arcade drive
     * Left Y controls forward and back Right X controls rotation
     * 
     * If want to auto align with cargo, you hold down left bumper
     * 
     * If want to auto align with vision tapes, you hold down right bumper
     * 
     * You still have forward and backwards control no matter what button you press
     */
    @Override
    public void teleopPeriodic() {
        // Change this to alter how quick or slow the feedback loop is
        double kP = 1.2;
 
        double forward = driverJoy.getY(leftSide);
        double turn = driverJoy.getX(rightSide);
 
        boolean cargoDesired = driverJoy.getBumper(leftSide);
        boolean tapeDesired = driverJoy.getBumper(rightSide);
        // If button 1 is pressed, then it will track cargo
        if (cargoDesired) {
 
            driveWanted.setBoolean(false);
            tapeWanted.setBoolean(false);
            cargoWanted.setBoolean(true);
            cargoSeen = cargoDetected.getBoolean(false);
 
            if (cargoSeen)
                targetAngle = cargoYaw.getDouble(0);
            else
                targetAngle = 0;
 
        } else if (tapeDesired) {
 
 
            driveWanted.setBoolean(false);
            tapeWanted.setBoolean(true);
            cargoWanted.setBoolean(false);
            // Checks if vision sees cargo or vision targets. This may not get called unless
            // cargo vision detected
            tapeSeen = tapeDetected.getBoolean(false);
 
            if (tapeSeen)
                targetAngle = tapeYaw.getDouble(0);
            else
                targetAngle = 0;
 
        } else {
 
 
            driveWanted.setBoolean(true);
            tapeWanted.setBoolean(false);
            cargoWanted.setBoolean(false);
 
            targetAngle = 0;
 
        }
        // Limit output to 0.3
        double output = limitOutput(-kP * targetAngle, 0.3);
 
        if (cargoDesired || tapeDesired)
            drive.arcadeDrive(forward, output);
        else
            drive.arcadeDrive(forward, turn);
 
    }
 
    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
 
    }
 
    public void disabledInit() {
 
    }
 
    public void disabledPeriodic() {
 
    }
 
    public double limitOutput(double number, double maxOutput) {
        if (number > 1.0) {
            number = 1.0;
        }
        if (number < -1.0) {
            number = -1.0;
        }
 
        if (number > maxOutput) {
            return maxOutput;
        }
        if (number < -maxOutput) {
            return -maxOutput;
        }
 
        return number;
    }
}
 
