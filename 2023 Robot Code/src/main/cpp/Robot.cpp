#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <photonlib/PhotonUtils.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/Compressor.h>
#include <frc/XboxController.h>
#include <iostream>
#include <string>
#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "AHRS.h"
#include <units/pressure.h>
#include <frc/DoubleSolenoid.h>




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

static const double kOffBalanceThresholdDegrees = 10.0f; //auto balance stuff
static const double kOnBalanceThresholdDegrees  = 5.0f;


class Robot : public frc::TimedRobot {

rev::CANSparkMax motor2{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor3{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor4{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor5{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor7{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motor8{8, rev::CANSparkMax::MotorType::kBrushless};

frc::Compressor phCompressor{15, frc::PneumaticsModuleType::REVPH};

frc::DoubleSolenoid intakeS{15, frc::PneumaticsModuleType::REVPH, 2, 3};
frc::DoubleSolenoid intakeN{15, frc::PneumaticsModuleType::REVPH, 4, 5};


double scale = 250, offset = -25;
//frc::AnalogPotentiometer pressureTransducer{1, scale, offset};

WPI_VictorSPX motor9{9};
WPI_VictorSPX motor10{10};
WPI_TalonSRX motor6{6};

frc::DifferentialDrive m_robotDrive{motor2, motor4};
frc::Joystick m_stickDrive{0};
frc::XboxController m_stickOperator{1};

rev::SparkMaxPIDController m_pidController = motor7.GetPIDController();
rev::SparkMaxRelativeEncoder motor8encoder = motor8.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42); //place holder for now to make sure the code was correct

rev::SparkMaxLimitSwitch forwardLimit = motor7.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
rev::SparkMaxLimitSwitch reverseLimit = motor8.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);

rev::SparkMaxRelativeEncoder motor2Encoder = motor2.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);
rev::SparkMaxRelativeEncoder motor4Encoder = motor4.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);


double drivedistance = 6;
double drivedistance2 = 7;
double drivedistance3 = -12; //if we are in the far RIGHT of BLUE
double drivedistanceB = 0.5; //pushing block/cone/whatever fowrard
double SelectedAuto = 0; //selected auto

AHRS *ahrs;

    bool autoBalanceXMode; //Auto balance stuff (not sure what it does)
    bool autoBalanceYMode;

     



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

  //static const double kOffBalanceThresholdDegrees = 10.0f;  //More navx stuff, it's all red so have fun with that
//static const double kOnBalanceThresholdDegrees = 5.0f;

//using namespace std;
    



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

    //phCompressor.SetClosedLoopControl(false);

    bool enabled = phCompressor.Enabled();
    bool pressureSwitch = phCompressor.GetPressureSwitchValue();
    //double current = phCompressor.GetCurrent();

    //frc::DoubleSolenoid intake{9, 4, 5}; //we are using double solenoids, this is supposed to define the intake one. idk why the name is red.
    
    m_robotDrive.SetDeadband(0);
    motor3.Follow(motor2);
    motor5.Follow(motor4);


    intakeS.Set(frc::DoubleSolenoid::Value::kOff); //These are supposed to be the different levels it goes or something.
    intakeS.Set(frc::DoubleSolenoid::Value::kForward);
    intakeS.Set(frc::DoubleSolenoid::Value::kReverse);

    // product-specific voltage->pressure conversion, see product manual
// in this case, 250(V/5)-25
// the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
// so if r is the raw AnalogPotentiometer output, the pressure is 250r-25



// scaled values in psi units
//double psi = pressureTransducer.Get();

    try
  {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi:
     * - Communication via USB.
     * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    ahrs = new AHRS(frc::SPI::Port::kMXP);
  }
  catch (std::exception &ex)
  {
    std::string what_string = ex.what();
    std::string err_msg("Error instantiating navX MXP:  " + what_string);
    const char * p_err_msg = err_msg.c_str();
  }
        autoBalanceXMode = false; //auto stuff
        autoBalanceYMode = false;




   // ahrs = new AHRS(SPI::Port::kMXP);  //I have no clue what any of this does, it was the only example code I could find. 
    //AHRS *ahrs; 

   // autoBalanceXMode = false;
 // autoBalanceYMode = false;

  };


  // Auto Area
  void AutonomousInit() override {

  motor2Encoder.SetPosition(0); 
  motor4Encoder.SetPosition(0);
  ahrs->Reset();

  drivedistance = (drivedistance * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); //converting from ft to counts of the encoder
  drivedistance2 = (drivedistance2 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); //converting from ft to counts of the encoder
  drivedistance = (drivedistance3 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 

  SelectedAuto = frc::SmartDashboard::GetNumber("Selected Auto", 0);

  }

  void AutonomousPeriodic() override {
   
    if(SelectedAuto == 1){  //just driving backwards


      if(motor2Encoder.GetPosition() < drivedistance){   //Driving to set distance

        motor2.Set(0.3);
      }

      else{

        motor2.Set(0);  //stopping once reaching set distance

      }

      if(motor4Encoder.GetPosition() < drivedistance){

        motor4.Set(0.3);
      }

     else{

      motor4.Set(0);

    }

  }

  if(SelectedAuto == 2){   //Auto balance

    if(motor2Encoder.GetPosition() < drivedistance){   //Driving to set distance

        motor2.Set(0.3);
      }

      else{

        motor2.Set(0);  //stopping once reaching set distance

      }

      if(motor4Encoder.GetPosition() < drivedistance){

        motor4.Set(0.3);
      }

     else{

      motor4.Set(0);

    }

  }

    if(SelectedAuto == 3){  //FAR RIGHT BLUE?????????? w/ block on ground level????? idk if it'll work but i tried

      if(motor2Encoder.GetPosition() < drivedistanceB){

        motor2.Set(0.3);
      
      }
      else{

        
        motor2.Set(0);
      
      }

      if(motor2Encoder.GetPosition() < drivedistance3){ 

        motor2.Set(0.3);
      }

      else{

        motor2.Set(0); 

      }

      if(motor4Encoder.GetPosition() < drivedistance3){

        motor4.Set(0.3);
      }

     else{

      motor4.Set(0);

    }

}

}
  // Teleop Area
  void TeleopInit() override {

    frc::SmartDashboard::PutNumber("Selected Auto", 0);

  }

  void TeleopPeriodic() override {
    //double StickX = Deadband(-m_stick.GetX(), 0.05, 2);
    //double StickY = Deadband(-m_stick.GetY(), 0.05, 2);

  m_robotDrive.ArcadeDrive(Deadband(-m_stickDrive.GetY(), 0.05, 2), Deadband(m_stickDrive.GetZ(), 0.05)); 


  
  if (m_stickOperator.GetRightBumperPressed()){
   motor9.Set(0.5); // When pressed the intake turns on
  }

   if (m_stickOperator.GetRightBumperReleased()) {
   motor9.Set(0); 
   } // When released the intake turns off

   if(m_stickOperator.GetRightBumperPressed()){
    motor10.Set(-0.5);
   }

   if(m_stickOperator.GetRightBumperReleased()){
    motor10.Set(0);
   }

  //pnuematics for INTAKEEEEEE
  intakeS.Set(frc::DoubleSolenoid::Value::kReverse);

if (m_stickOperator.GetXButtonPressed()) {
   intakeS.Toggle();
}


  intakeN.Set(frc::DoubleSolenoid::Value::kReverse);

if (m_stickOperator.GetXButtonPressed()) {
   intakeN.Toggle();
}
//end pnuematics

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
    frc::SmartDashboard::PutNumber("ProcessVariable", motor8encoder.GetPosition());

    frc::SmartDashboard::PutBoolean("Forward Limit Switch", forwardLimit.Get());
    frc::SmartDashboard::PutBoolean("Reverse Limit Switch", forwardLimit.Get());
    //end spark max limit switch

 //bool motionDetected = ahrs->IsMoving();  //More code from the only example of navx code I could find, I am going crazy rn
  //SmartDashboard::PutBoolean("MotionDetected", motionDetected);

    frc::SmartDashboard::PutNumber("NavX Test", ahrs->GetAngle());


};

 
};  

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(StickY, StickX);
  


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
