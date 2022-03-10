// package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// // import com.revrobotics.CANSparkMax;
// // import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// // import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// public class Limelight extends TimedRobot {
//     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//     // CANSparkMax lFMotor = new CANSparkMax(4, MotorType.kBrushless);
//     // CANSparkMax lBMotor = new CANSparkMax(8, MotorType.kBrushless);
//     // MotorControllerGroup lMotors = new MotorControllerGroup(DriveMethod.lFMotor, DriveMethod.lBMotor);

//     // CANSparkMax rFMotor = new CANSparkMax(1, MotorType.kBrushless);
//     // CANSparkMax rBMotor = new CANSparkMax(12, MotorType.kBrushless);
//     // MotorControllerGroup rMotors = new MotorControllerGroup(rFMotor, rBMotor);
//     // MecanumDrive Drive = new MecanumDrive(DriveMethod.lFMotor, DriveMethod.lBMotor, DriveMethod.rFMotor, DriveMethod.rBMotor);
//     // DriveMethod.mDrive MecanumDrive = new DriveMethod.mDrive();
    
//     public double LimelightMain() {
//         // rMotors.setInverted(true);
//         NetworkTableEntry tx = table.getEntry("tx");
//         NetworkTableEntry ty = table.getEntry("ty");
//         NetworkTableEntry ta = table.getEntry("ta");
//         double x = tx.getDouble(0.0);
//         double y = ty.getDouble(0.0);
//         double area = ta.getDouble(0.0);
//         // double xSpeed = x / 100;

//         SmartDashboard.putNumber("LimelightX", x);
//         SmartDashboard.putNumber("LimelightY", y);
//         SmartDashboard.putNumber("LimelightArea", area);
//         // Drive.driveCartesian(0, 0, 0.1);
//         // if (Math.abs(x) > 2) {
//         //     Drive.driveCartesian(0, 0, .5);
//         // }
//         return x;
//     }
// }
