#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <photonlib/PhotonUtils.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"


// Returns a 1 for postive numbers and a -1 for negative numbers
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

 //WPI_VictorSPX m_motor{2};
  //WPI_TalonSRX  m_motor{2};
frc::DifferentialDrive m_robotDrive{motor2, motor4};
  frc::Joystick m_stick{0};

 public:
  void RobotInit() override {
    motor2.RestoreFactoryDefaults();
    motor3.RestoreFactoryDefaults();
    motor4.RestoreFactoryDefaults();
    motor5.RestoreFactoryDefaults();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    //m_rightMotor.SetInverted(true);

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

    m_robotDrive.TankDrive(-m_stick.GetY(), m_stick.GetZ());
  

  //if (m_stick.GetRawButtonPressed(1)) {
  // turnIntakeOn(); // When pressed the intake turns on   (need to tweak)

};
 // if (m_stick.GetRawButtonReleased(0)) {
  // turnIntakeOff(); // When released the intake turns off
};
  

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(StickY, StickX);
  


  // Test Methods
  void TestInit() override{ }
  void TestPeriodic() override{ }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
