package frc.robot;

//Imports
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drive extends TimedRobot {
    AHRS navX = new AHRS(SPI.Port.kMXP);
    //Intializing Left Motor
    public CANSparkMax lFMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax lBMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private MotorControllerGroup lMotors = new MotorControllerGroup(lFMotor, lBMotor);

    //Initializing Right Motors
    public CANSparkMax rFMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax rBMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    public TalonSRX climber = new TalonSRX(9);
    public MotorControllerGroup rMotors = new MotorControllerGroup(rFMotor, rBMotor);
    public MotorControllerGroup lMotors = new MotorControllerGroup(lFMotor, lBMotor);

    PneumaticsControlModule PCM1 = new PneumaticsControlModule(0);
    PneumaticsControlModule PCM2 = new PneumaticsControlModule(1);
 
    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid drivePistons = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);

    // private Solenoid rWSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    // private Solenoid lWSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

    boolean tank = false;
    boolean down = true;
    double climberM = 0.0;
    //Initializing NonMotors
    MecanumDrive mDrive = new MecanumDrive(lFMotor, lBMotor, rFMotor, rBMotor);
    DifferentialDrive tDrive = new DifferentialDrive(lMotors, rMotors);
    
    XboxController Xbox1 = new XboxController(0);
    private int kJoystickChannel;
    Joystick mStick = new Joystick(kJoystickChannel);
    // Limelight limelight = new Limelight();\
    public void DriveInit() {
        rFMotor.setInverted(true);
        rBMotor.setInverted(true);
    }

    public void DriveMain() {
        //Inverts Right Motors To Ensure They Go The Correct Way
        if (Xbox1.getYButton()) {
            climberM = 0.3;
        }

        //Tells the Robot How Much Power to Apply to the Motors Based on Controller Inputs
        if (tank && !down){ //Takes in boolean and switches solenoid output based on it. 
            drivePistons.set(Value.kForward);
            down = true;
        }
        else if (!tank && down) {
            drivePistons.set(Value.kReverse);
            down = false;
        }
        if (tank) {
            tDrive.arcadeDrive(-Xbox1.getLeftY(), Xbox1.getRightX());
            climber.set(ControlMode.PercentOutput, climberM);
            //Puts Power of Each Motor Into Smart Dashboard
            SmartDashboard.putNumber("L Power", lMotors.get());
            SmartDashboard.putNumber("R Power", rMotors.get());
        }
        else if (!tank) {
            mDrive.driveCartesian(-Xbox1.getLeftY(), Xbox1.getLeftX(), Xbox1.getRightX());

            //Puts Power of Each Motor Into Smart Dashboard
            SmartDashboard.putNumber("LF Power", lFMotor.get());
            SmartDashboard.putNumber("LB Power", lBMotor.get());
            SmartDashboard.putNumber("RF Power", rFMotor.get());
            SmartDashboard.putNumber("RB Power", rBMotor.get());
        } 
        else if (!tank) {
            mDrive.setSafetyEnabled(true);
            while (isTeleop() && isEnabled()) {
                if ( Xbox1.getRightBumperPressed()) {
                    navX.reset();
                }
                mDrive.driveCartesian(Xbox1.getLeftX(), Xbox1.getLeftY(), Xbox1.getRightX(), navX.getAngle());
                Timer.delay(0.005);
            }
        }
        
    }

    public static void driveCartesian(int i, int j, double d) {
    }

    // public void LimelightDrive() {
    //     // Drive.driveCartesian(0, 0, 0.1);
    //     if (Math.abs(limelight.LimelightMain()) > 2) {
    //         Drive.driveCartesian(0, 0, .1);
    //     }
    // }
}
