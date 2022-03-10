// package frc.robot;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
// import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
// import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// // import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// // import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Odometry {
//     public static final double kMaxSpeed = 3.0;
//     public static final double kMaxAngularSpeed = Math.PI;
    
//     //Intializing Left Motor
//     public final CANSparkMax lFMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
//     public final CANSparkMax lBMotor = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

//     //Initializing Right Motors
//     public final CANSparkMax rFMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
//     public final CANSparkMax rBMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
//     MecanumDrive drive = new MecanumDrive(lFMotor, lBMotor, rFMotor, rBMotor);

//     //Initializing Encoders
//     private final Encoder lFEncoder = new Encoder(0, 1);
//     private final Encoder lBEncoder = new Encoder(2, 3);
//     private final Encoder rFEncoder = new Encoder(4, 5);
//     private final Encoder rBEncoder = new Encoder(6, 7);

//     //Initializing location reletive to rest of robot
//     private final Translation2d lFLocation = new Translation2d(0.381, 0.381);
//     private final Translation2d lBLocation = new Translation2d(0.381, -0.381);
//     private final Translation2d rFLocation = new Translation2d(-0.381, 0.381);
//     private final Translation2d rBLocation = new Translation2d(-0.381, -0.381);

//     //Initializing PID Controllers
//     private final PIDController lFPidController = new PIDController(1, 0, 0);
//     private final PIDController lBPidController = new PIDController(1, 0, 0);
//     private final PIDController rFPidController = new PIDController(1, 0, 0);
//     private final PIDController rBPidController = new PIDController(1, 0, 0);

//     //Initializing Gyro
//     private final AnalogGyro gyro = new AnalogGyro(0);

//     //Combining locations into 1 variable
//     private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(lFLocation, rFLocation, lBLocation, rBLocation);

//     //Combining kinematics and gyro value into 1 variable
//     private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2d());

//     //Initializing speed variable
//     private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

//     public void Drivetrain() {
//         //Resets gyro so it doesn't start off
//         gyro.reset();
//         //Inverting r motors so their voltages are positive
//         rFMotor.setInverted(true);
//         rBMotor.setInverted(true);
//     }

//     public MecanumDriveWheelSpeeds getCurrentState() {
//         //Says how fast each motor is traveling
//         return new MecanumDriveWheelSpeeds(lFEncoder.getRate(), rFEncoder.getRate(), lBEncoder.getRate(), rBEncoder.getRate());
//     }

//     public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
//         //Calculates speed for each motor
//         final double lFFeedforward = feedforward.calculate(speeds.frontLeftMetersPerSecond);
//         final double rFFeedforward = feedforward.calculate(speeds.frontRightMetersPerSecond);
//         final double lBFeedforward = feedforward.calculate(speeds.rearLeftMetersPerSecond);
//         final double rBFeedForward = feedforward.calculate(speeds.rearRightMetersPerSecond);

//         //Info each motor tells us
//         final double lFOutput = lFPidController.calculate(lFEncoder.getRate(), speeds.frontLeftMetersPerSecond);
//         final double rFOutput = rFPidController.calculate(rFEncoder.getRate(), speeds.frontRightMetersPerSecond);
//         final double lBOutput = lBPidController.calculate(lBEncoder.getRate(), speeds.rearLeftMetersPerSecond);
//         final double rBOutput = rBPidController.calculate(rBEncoder.getRate(), speeds.rearRightMetersPerSecond);

//         //Sets Voltage for each motor
//         lFMotor.setVoltage(lFOutput + lFFeedforward);
//         rFMotor.setVoltage(rFOutput + rFFeedforward);
//         lBMotor.setVoltage(lBOutput + lBFeedforward);
//         rBMotor.setVoltage(rBOutput + rBFeedForward);
//     }
//     //Ensures we don't get an error
//     @SuppressWarnings("ParameterName")
//     //Final drive method
//     public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
//         var mecanumDriveWheelSpeeds = kinematics.toWheelSpeeds(fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()): new ChassisSpeeds(xSpeed, ySpeed, rot));
//         mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
//         setSpeeds(mecanumDriveWheelSpeeds);
//     }

//     //Updates odometry often
//     public void updateOdometry() {
//         odometry.update(gyro.getRotation2d(), getCurrentState());
//         SmartDashboard.putString("test", "yes");
//     }
// }