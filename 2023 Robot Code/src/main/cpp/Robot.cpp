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
  frc::PWMSparkMax m_leftMotor{9};
  frc::PWMSparkMax m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::Joystick m_stickDrive{0};
  //frc::Joystick m_stickOperator{1}; Adding second joystick

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);

  }


  // Auto Area
  void AutonomousInit() override {

  }

  void AutonomousPeriodic() override {
   
  }


  // Teleop Area
  void TeleopInit() override {

  }

  void TeleopPeriodic() override {
    double StickX = Deadband(-m_stickDrive.GetX(), 0.05, 2);
    double StickY = Deadband(-m_stickDrive.GetY(), 0.05, 2);

  //if (m_stickOperator.GetRawButtonPressed(1)) {
  // turnIntakeOn(); // When pressed the intake turns on   (need to tweak)

}
 // if (m_stickOperator.GetRawButtonReleased(0)) {
  // turnIntakeOff(); // When released the intake turns off
}



    // Drive with arcade style
    m_robotDrive.ArcadeDrive(StickY, StickX);
  }


  // Test Methods
  void TestInit() override {

  }

  void TestPeriodic() override {

  }

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
