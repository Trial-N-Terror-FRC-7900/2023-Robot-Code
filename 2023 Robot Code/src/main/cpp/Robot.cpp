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
#include <iostream>

#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"


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

rev::CANSparkMax rightLeadmotor{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rightFollowmotor{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax leftLeadmotor{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax leftFollowmotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax armRotate{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax armRotate2{8, rev::CANSparkMax::MotorType::kBrushless};

frc::Compressor phCompressor{15, frc::PneumaticsModuleType::REVPH};

frc::DoubleSolenoid intakeS{15, frc::PneumaticsModuleType::REVPH, 2, 3};
frc::DoubleSolenoid intakeN{15, frc::PneumaticsModuleType::REVPH, 4, 5};


CANdle candle{15};
//void candlePurple(){
        //candle.SetLEDs(75,0,130);}
//void candleGreen(){
        //candle.SetLEDs(0, 119, 20);}
//Animation(1, 0.5, 64, 0);

double scale = 250, offset = -25;
//frc::AnalogPotentiometer pressureTransducer{1, scale, offset};

WPI_VictorSPX intakemotorF{9};
WPI_VictorSPX intakemotorR{10}; 
WPI_VictorSPX handF{13};
WPI_VictorSPX handR{14}; 
WPI_TalonFX armExtend{6};



std::string _sb;
int kPIDLoopIdx;
bool kTimeoutMs;
int targetPositionRotations;
int _loops = 0;
bool _lastButton1 = false;

frc::DifferentialDrive m_robotDrive{rightLeadmotor, leftLeadmotor};
frc::Joystick m_stickDrive{0};
frc::XboxController m_stickOperator{1};

rev::SparkMaxPIDController m_pidController = armRotate.GetPIDController();
rev::SparkMaxRelativeEncoder motor8encoder = armRotate2.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42); //place holder for now to make sure the code was correct

rev::SparkMaxLimitSwitch forwardLimit = armRotate.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
rev::SparkMaxLimitSwitch reverseLimit = armRotate2.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);

rev::SparkMaxRelativeEncoder rightEncoder = rightLeadmotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);
rev::SparkMaxRelativeEncoder leftEncoder = leftLeadmotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);


double drivedistance = 6.3662; //120 inches i beleive
double drivedistance2 = 3.81972; //72 inches
double drivedistance3 = -12; //if we are in the far RIGHT of BLUE
double drivedistance4 = 0.5; //pushing block/cone/whatever fowrard MAYBEEEEEEEEEEE
double SelectedAuto = 0; //selected auto


int autoStep = 0;

AHRS *ahrs;

    bool autoBalanceXMode; //Auto balance stuff (not sure what it does)
    bool autoBalanceYMode;



  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

  //static const double kOffBalanceThresholdDegrees = 10.0f;  //More navx stuff, it's all red so have fun with that
//static const double kOnBalanceThresholdDegrees = 5.0f


 public: 
  void RobotInit() override {
    rightLeadmotor.RestoreFactoryDefaults();
    rightFollowmotor.RestoreFactoryDefaults();
    leftLeadmotor.RestoreFactoryDefaults();
    leftFollowmotor.RestoreFactoryDefaults();
    intakemotorF.ConfigFactoryDefault();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightLeadmotor.SetInverted(true);

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

    frc::SmartDashboard::PutNumber("Selected Auto", 0);

    //Spark Maxes limit switches
    forwardLimit.EnableLimitSwitch(false);
    reverseLimit.EnableLimitSwitch(false);
    frc::SmartDashboard::PutBoolean("Forward Limit Enabled", forwardLimit.IsLimitSwitchEnabled());
    frc::SmartDashboard::PutBoolean("Reverse Limit Enabled", reverseLimit.IsLimitSwitchEnabled());

    //phCompressor.SetClosedLoopControl(false);

    bool enabled = phCompressor.Enabled();
    bool pressureSwitch = phCompressor.GetPressureSwitchValue();
    //double current = phCompressor.GetCurrent();

    
    m_robotDrive.SetDeadband(0);
    rightFollowmotor.Follow(rightLeadmotor);
    leftFollowmotor.Follow(leftLeadmotor);


    intakeS.Set(frc::DoubleSolenoid::Value::kOff); //These are supposed to be the different levels it goes or something.
    intakeS.Set(frc::DoubleSolenoid::Value::kForward);
    intakeS.Set(frc::DoubleSolenoid::Value::kReverse);

    int _loops = 0;
	  bool _lastButton1 = false;
	  /** save the target position to servo to */
	  double targetPositionRotations;

    /* Factory Default all hardware to prevent unexpected behaviour */
		armExtend.ConfigFactoryDefault();


    /**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = armExtend.GetSensorCollection().GetIntegratedSensorAbsolutePosition();
		/* use the low level API to set the quad encoder signal */
		armExtend.SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		armExtend.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		armExtend.SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		armExtend.ConfigNominalOutputForward(0, kTimeoutMs);
		armExtend.ConfigNominalOutputReverse(0, kTimeoutMs);
		armExtend.ConfigPeakOutputForward(1, kTimeoutMs);
		armExtend.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		armExtend.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		armExtend.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		armExtend.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		armExtend.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    CANdleConfiguration config;

  config.stripType = LEDStripType::RGB;

  config.brightnessScalar = 0.5; 

  candle.ConfigAllSettings(config);
  

  //RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  //candle.Animate(*rainbowAnim);

  //candle.SetLEDs(225, 225, 225);
  //virtual  Animation(1, 0.5, 64);


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
        autoBalanceXMode = false; //auto balance stuff
        autoBalanceYMode = false;




   // ahrs = new AHRS(SPI::Port::kMXP);  //I have no clue what any of this does, it was the only example code I could find. 
    //AHRS *ahrs; 


  };


  // Auto Area
  void AutonomousInit() override {

  rightEncoder.SetPositionConversionFactor(5.28309);
  leftEncoder.SetPositionConversionFactor(5.28309);
  rightEncoder.SetPosition(0); 
  leftEncoder.SetPosition(0);
      (ahrs->IsCalibrating());
  ahrs->Reset();
  //ahrs->Reset(); <-crashes the code?

 // drivedistance = (drivedistance * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance2 = (drivedistance2 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance = (drivedistance3 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 


  SelectedAuto = frc::SmartDashboard::GetNumber("Selected Auto", 0);

  //RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  //candle.Animate(*rainbowAnim);
  }

  void AutonomousPeriodic() override {
 //  RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
 // candle.Animate(*rainbowAnim);
  if(SelectedAuto == 1){  //just driving 10ft, with autobalance?

    frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
    if(rightEncoder.GetPosition() < drivedistance){ 
      m_robotDrive.ArcadeDrive(0.4, 0); 
    }
    else{
      m_robotDrive.ArcadeDrive(0, 0); 
    }




  }

  if(SelectedAuto == 2){   //drive different distance, about 6ft, to get onto platform?

  switch(autoStep) {
    case 0:
     frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
     if(rightEncoder.GetPosition() < 10){ 
      m_robotDrive.ArcadeDrive(0.8, 0); 
     }
     else{
      m_robotDrive.ArcadeDrive(0, 0); 
      autoStep = 1;
      rightEncoder.SetPosition(0); 
      leftEncoder.SetPosition(0);
     }
    case 1:
     frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
     if(rightEncoder.GetPosition() > -5){ 
      m_robotDrive.ArcadeDrive(-0.8, 0); 
     }
     else{
      m_robotDrive.ArcadeDrive(0, 0); 
      autoStep = 2;
      rightEncoder.SetPosition(0); 
      leftEncoder.SetPosition(0);
     }
    case 2:
     if(fabs(rightEncoder.GetPosition()) < 2){
      m_robotDrive.ArcadeDrive(0.005*(-Deadband(ahrs->GetPitch(), 5)), 0);

     }

     

}


     // try {
     // Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
    //   m_robotDrive.DriveCartesian(xAxisRate, yAxisRate, m_stickDrive.GetZ());
    //  } 
    //  catch (exception& ex ) {
    //   string err_string = "Drive system error:  ";
    //   err_string += ex.what();
    //   DriverStation::ReportError(err_string.c_str());
    //  }
     // Wait(0.005); // wait 5ms to avoid hogging CPU cycle

  };

    if(SelectedAuto == 3){  //FAR RIGHT BLUE w/ block on ground level????? idk if it'll work but i tried, i need a thing thats like do this and then this.

      if(rightEncoder.GetPosition() < drivedistance3){

        rightLeadmotor.Set(0.3);
      
      }
      else{

        
        rightLeadmotor.Set(0);
      
      }

       if(leftEncoder.GetPosition() < drivedistance3){

        leftLeadmotor.Set(0.3);
      
      }
      else{

        
        leftLeadmotor.Set(0);
      
      }

      if(rightEncoder.GetPosition() < drivedistance4){ 

        rightLeadmotor.Set(0.3);
      }

      else{

        rightLeadmotor.Set(0); 

      }

      if(leftEncoder.GetPosition() < drivedistance4){

        leftLeadmotor.Set(0.3);
      }

     else{

      leftLeadmotor.Set(0);

    }

}

}
  // Teleop Area
  void TeleopInit() override {

    

  }

  void TeleopPeriodic() override {
    //double StickX = Deadband(-m_stick.GetX(), 0.05, 2);
    //double StickY = Deadband(-m_stick.GetY(), 0.05, 2); 

     double xAxisRate = m_stickDrive.GetX();   //Balancing code?
     double yAxisRate = m_stickDrive.GetY();
     double pitchAngleDegrees = ahrs->GetPitch();
     double rollAngleDegrees = ahrs->GetRoll();

      if ( !autoBalanceXMode &&
       (fabs(pitchAngleDegrees) >=
       fabs(kOffBalanceThresholdDegrees))) {
       autoBalanceXMode = true;
      }
      else if ( autoBalanceXMode &&
        (fabs(pitchAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
        autoBalanceXMode = false;
      }
      if ( !autoBalanceYMode &&
        (fabs(pitchAngleDegrees) >=
        fabs(kOffBalanceThresholdDegrees))) {
        autoBalanceYMode = true;
      }
      else if ( autoBalanceYMode &&
        (fabs(pitchAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
        autoBalanceYMode = false;
      }

      if ( autoBalanceXMode ) {
       double pitchAngleRadians = pitchAngleDegrees * (M_PI / 180.0);
        xAxisRate = sin(pitchAngleRadians) * -1;
      }
      if ( autoBalanceYMode ) {
       double rollAngleRadians = rollAngleDegrees * (M_PI / 180.0);
       yAxisRate = sin(rollAngleRadians) * -1;
      }

  
 // if (m_stickOperator.GetRightBumperPressed()){
  // intakemotorF.Set(0.5); // When pressed the intake turns on
  //}

 //  if (m_stickOperator.GetRightBumperReleased()) {
  // intakemotorF.Set(0); 
  // } // When released the intake turns off

  // if(m_stickOperator.GetRightBumperPressed()){
  //  intakemotorR.Set(-0.5);
 //  }

 //  if(m_stickOperator.GetRightBumperReleased()){
 //   intakemotorR.Set(0);
//   }

   if(m_stickDrive.GetRawButton(1)){
    m_robotDrive.ArcadeDrive(0.2*(Deadband(-m_stickDrive.GetY(), 0.05, 2)), 0.5*Deadband(-m_stickDrive.GetZ(), 0.05, 2));

   }
   else{
    m_robotDrive.ArcadeDrive(Deadband(-m_stickDrive.GetY(), 0.05, 2), 0.5*Deadband(-m_stickDrive.GetZ(), 0.05, 2)); 

    }

  //pnuematics for INTAKEEEEEE
//intakeS.Set(frc::DoubleSolenoid::Value::kReverse);
 //   if (m_stickOperator.GetXButtonPressed()) {
 //  intakeS.Toggle();
//}

//intakeN.Set(frc::DoubleSolenoid::Value::kReverse);
//    if (m_stickOperator.GetXButtonPressed()) {
//   intakeN.Toggle();
//}
//end pnuematics, when pressed it will either go up or go down, depending on current orientation 


  if (m_stickOperator.GetLeftBumperPressed()){
   handF.Set(0.5); // When pressed the intake turns on
  }

   if (m_stickOperator.GetLeftBumperReleased()) {
   handF.Set(0); 
   } // When released the intake turns off

   if(m_stickOperator.GetLeftBumperPressed()){
    handR.Set(-0.5);
   }

   if(m_stickOperator.GetLeftBumperReleased()){
    handR.Set(0);
   }



   if(m_stickDrive.GetRawButton(7)){
      candle.ClearAnimation(0);
      }
   if(m_stickDrive.GetRawButtonPressed(8)){
      RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
        candle.Animate(*rainbowAnim);
        frc::SmartDashboard::PutNumber("candle", 2);
      }

   if(m_stickDrive.GetRawButton(9)){
    candle.SetLEDs(0, 119, 20);
   }

   if(m_stickDrive.GetRawButton(10)){
    candle.SetLEDs(75,0,130);
   }
   if(m_stickDrive.GetRawButton(11)){
    candle.SetLEDs(255,255,0);
   }



		double leftYstick = m_stickOperator.GetYButtonPressed();
		double motorOutput = armExtend.GetMotorOutputPercent();
		bool button2 = m_stickOperator.GetRightBumperPressed();
	
		//press button and ot'll go where you want it to go
		if (m_stickOperator.GetYButtonPressed()) {
			armExtend.Set(ControlMode::Position, 5000); 
		}

    if (m_stickOperator.GetXButtonPressed()) {
		armExtend.Set(ControlMode::Position, 4500); 
		}

    if (m_stickOperator.GetAButtonPressed()) {
		armExtend.Set(ControlMode::Position, 4000); 
		}
		//override, add deadband for xbox controller 
		if (m_stickOperator.GetRightBumperPressed()) {
			armExtend.Set(ControlMode::PercentOutput, Deadband(leftYstick, 0.01));
		}


  


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
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max);
    }
      kMinOutput = min; kMaxOutput = max;
    m_pidController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
  //end PID control
    
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
