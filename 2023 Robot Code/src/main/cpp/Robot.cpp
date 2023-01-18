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
rev::CANSparkMax testmotor{3, rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax motor2{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor3{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor4{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor5{5, rev::CANSparkMax::MotorType::kBrushless};

 //WPI_VictorSPX m_motor{2};
  //WPI_TalonSRX  m_motor{2};
//frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::Joystick m_stick{0};

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
<<<<<<< Updated upstream
    m_rightMotor.SetInverted(true);

=======
>>>>>>> Stashed changes
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
    //double StickX = Deadband(-m_stick.GetX(), 0.05, 2);
    //double StickY = Deadband(-m_stick.GetY(), 0.05, 2);

    testmotor.Set(m_stick.GetY());
    motor2.Set(m_stick.GetY());
    motor3.Follow(motor2);
    motor4.Follow(motor2);
    motor5.Follow(motor2);

    motor2.Set(-m_stick.GetZ());
    motor3.Follow(motor2);
    motor4.Set(m_stick.GetZ());
    motor5.Follow(motor4);

    //testmotor.Set(m_stick.GetZ());
    //testmotor.Set(-m_stick.GetZ());

<<<<<<< Updated upstream
  //if (m_stick.GetRawButtonPressed(1)) {
  // turnIntakeOn(); // When pressed the intake turns on   (need to tweak)

}
 // if (m_stick.GetRawButtonReleased(0)) {
  // turnIntakeOff(); // When released the intake turns off
}



=======
  
>>>>>>> Stashed changes
    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(StickY, StickX);
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
