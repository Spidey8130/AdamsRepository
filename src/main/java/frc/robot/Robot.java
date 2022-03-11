// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private final XboxController controller = new XboxController(0);
  // private final Odometry mecanum = new Odometry();
  // private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Drive DriveMethod = new Drive();
  // private Climber ClimberMethod = new Climber();
  // private Limelight LimelightMethod = new Limelight();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    DriveMethod.DriveInit();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        // driveWithJoystick(false);
        //Updates odometry every iteration
        // mecanum.updateOdometry();
        // mecanum.lBMotor.set(0.1);
        // LimelightMethod.LimelightMain();
        // DriveMethod.LimelightDrive();
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        SmartDashboard.putString("Test", "test");
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // mecanum.rFMotor.setInverted(true);
    // mecanum.rBMotor.setInverted(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Tank On?", DriveMethod.tank);
    // mecanum.lFMotor.set(controller.getLeftY());
    // mecanum.Drivetrain();
    // mecanum.updateOdometry();
    // mecanum.drive.driveCartesian(0.0, 0.0, 0.2);
    // driveWithJoystick(true);
    // mecanum.drive.driveCartesian(-controller.getLeftY(), controller.getLeftX(), controller.getRightX());
    // mecanum.lBMotor.set(controller.getLeftY());
    tankDrop();
    DriveMethod.DriveMain();
    // ClimberMethod.Up();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

//   private void driveWithJoystick(boolean fieldRelative) {
//     //Defining variables to say how much power to apply
//     if (fieldRelative) {
//       // mecanum.Drivetrain();
//       final var xSpeed = -xSpeedLimiter.calculate(controller.getLeftX()) * Odometry.kMaxSpeed;
//       final var ySpeed = ySpeedLimiter.calculate(controller.getLeftY()) * Odometry.kMaxSpeed;
//       final var rot = rotLimiter.calculate(controller.getRightX()) * Odometry.kMaxAngularSpeed;
//       SmartDashboard.putNumber("xspeed", xSpeed);
//       SmartDashboard.putNumber("yspeed", ySpeed);
//       SmartDashboard.putNumber("rot", rot);

//       //Makes robot drive based on values
//       // mecanum.drive(ySpeed, xSpeed, rot, fieldRelative);
//       // mecanum.drive.driveCartesian(ySpeed, xSpeed, rot);
//       // mecanum.rFMotor.setInverted(true);
//       // mecanum.rBMotor.setInverted(true);
//       mecanum.drive.driveCartesian(controller.getLeftY(), controller.getLeftX(), controller.getRightX());
//     }
//   }
    public void tankDrop() {
      if(DriveMethod.Xbox1.getBButtonPressed()) {
        DriveMethod.tank = true;
      } else if (DriveMethod.Xbox1.getAButtonPressed()) {
        DriveMethod.tank = false;
      }
    }
}
