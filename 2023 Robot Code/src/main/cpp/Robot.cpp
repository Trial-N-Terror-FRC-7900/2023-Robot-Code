#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <photonlib/PhotonUtils.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/Compressor.h>


// Returns a 1 for posghtive numbers and a -1 for negative numbers
template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Proper Deadband Code to correctly scale the input
double Deadband(double input, double limit){
  double output;

  if( fabs(input) < limit){
    output = 0;
  }
  else{
    output = (1 / (1 - limit)) * (input + (-sgn(input) * limit));
  }


  return output;
}
// Proper Deadband Code but with power scaling
double Deadband(double input, double limit, int power_scale){
  double output;

  if( fabs(input) < limit){
    output = 0;
  }
  else{
    output = (1 / (1 - limit)) * (input + (-sgn(input) * limit));
  }
  if(power_scale == 2){
    output = output * output;
    output = sgn(input) * output;
  }
  if(power_scale == 3){
    output = output * output * output;
  }

  return output;
}

class Robot : public frc::TimedRobot {


rev::CANSparkMax motor2{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor3{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor4{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor5{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor7{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor8{8, rev::CANSparkMax::MotorType::kBrushless};

frc::Compressor phCompressor{6, frc::PneumaticsModuleType::REVPH};

WPI_VictorSPX motor9{9};
WPI_TalonSRX motor6{6};

frc::DifferentialDrive m_robotDrive{motor2, motor4};
frc::Joystick m_stickDrive{0};
frc::Joystick m_stickOperator{1};

rev::SparkMaxPIDController m_pidController = motor7.GetPIDController();
rev::SparkMaxRelativeEncoder m_encoder = motor8.GetEncoder(); //wasnt sure what were using as an encoder yet, this is just a place holder for now to make sure the code was correct

rev::SparkMaxLimitSwitch forwardLimit = motor7.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
rev::SparkMaxLimitSwitch reverseLimit = motor8.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);

//frc::sim::SingleJointedArmSim ArmSim{motor6,
                                     // kArmGearing,
                                     // kCarriageMass,
                                      //kArmDrumRadius,
                                     // kMinArmHeight,
                                     // kMaxArmHeight,
                                     // true,
                                     // {0.01}};
  //frc::sim::EncoderSim m_encoderSim{m_encoder};

  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;



 public: 
  void RobotInit() override {
    motor2.RestoreFactoryDefaults();
    motor3.RestoreFactoryDefaults();
    motor4.RestoreFactoryDefaults();
    motor5.RestoreFactoryDefaults();
    motor9.ConfigFactoryDefault();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    //m_rightMotor.SetInverted(true);

    //PID start
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);
    //PID end

    //Spark Maxes limit switches
    forwardLimit.EnableLimitSwitch(false);
    reverseLimit.EnableLimitSwitch(false);
    frc::SmartDashboard::PutBoolean("Forward Limit Enabled", forwardLimit.IsLimitSwitchEnabled());
    frc::SmartDashboard::PutBoolean("Reverse Limit Enabled", reverseLimit.IsLimitSwitchEnabled());

    motor3.Follow(motor2);
    motor5.Follow(motor4);



  };


  // Auto Area
  void AutonomousInit() override {

  }

  void AutonomousPeriodic() override {
   
    
    

  }


  // Teleop Area
  void TeleopInit() override {

  }

  void TeleopPeriodic() override {
    //double StickX = Deadband(-m_stick.GetX(), 0.05, 2);
    //double StickY = Deadband(-m_stick.GetY(), 0.05, 2);

    m_robotDrive.TankDrive(-m_stickDrive.GetY(), m_stickDrive.GetZ());
  
  if (m_stickOperator.GetRawButtonPressed(2)){
   motor9.Set(0.5); // When pressed the intake turns on
  }

   if (m_stickOperator.GetRawButtonReleased(2)) {
   motor9.Set(0); // When released the intake turns off

  //start PID control
  //hi scarlette this is for the driver station thing 
   double p = frc::SmartDashboard::GetNumber("P Gain", 0);
   double i = frc::SmartDashboard::GetNumber("I Gain", 0);
   double d = frc::SmartDashboard::GetNumber("D Gain",0);
   double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
   double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
   double max = frc::SmartDashboard::GetNumber("Max Output", 0);
   double min = frc::SmartDashboard::GetNumber("Min Output", 0);
   double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { }
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
  m_pidController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
  //end PID control

  //start spark max limit switch/arm rotation limit switch
    forwardLimit.EnableLimitSwitch(frc::SmartDashboard::GetBoolean("Forward Limit Enabled", false));
    reverseLimit.EnableLimitSwitch(frc::SmartDashboard::GetBoolean("Reverse Limit Enabled", false));
    
    frc::SmartDashboard::PutNumber("SetPoint", rotations);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());

    frc::SmartDashboard::PutBoolean("Forward Limit Switch", forwardLimit.Get());
    frc::SmartDashboard::PutBoolean("Reverse Limit Switch", forwardLimit.Get());
    //end spark max limit switch



};

};

 
};  

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(StickY, StickX);
  


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
