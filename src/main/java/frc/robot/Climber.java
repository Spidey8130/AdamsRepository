// package frc.robot;

// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Climber extends TimedRobot {
//     private DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);

//     public double climberPosition;

//     public XboxController Xbox1 = new XboxController(0);

//     public boolean pistonsUp;

//     public boolean arePistonsUp() {
//         if(Xbox1.getAButtonReleased()){
//             pistonsUp = true;
//         } else {
//             pistonsUp = false;
//         }
//         return pistonsUp;
//     }

//     public void Up() {
//         if (pistons.get() == Value.kForward){
//             SmartDashboard.putBoolean("Pistons Down", true);
//         } else if (pistons.get() == Value.kReverse){
//             SmartDashboard.putBoolean("Pistons Down", false);
//         }

        
//         if (pistonsUp) {
//             pistons.set(Value.kForward);
//         } else {
//             pistons.set(Value.kReverse);
//         }

//         SmartDashboard.putBoolean("Pistons up?", pistonsUp);
//     }
// }
