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


double scale = 250, offset = -25;
//frc::AnalogPotentiometer pressureTransducer{1, scale, offset};

WPI_VictorSPX intakemotorF{9};
WPI_VictorSPX intakemotorR{10}; 
WPI_VictorSPX handF{13};
WPI_VictorSPX handR{14}; 
WPI_TalonSRX armExtend{6};

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


double drivedistance = 6;
double drivedistance2 = 7;
double drivedistance3 = -12; //if we are in the far RIGHT of BLUE
double drivedistance4 = 0.5; //pushing block/cone/whatever fowrard MAYBEEEEEEEEEEE
double SelectedAuto = 0; //selected auto


int switchOne = 0; //not completed? Needs work I believe.
int switchTwo = 2;
int switchThree = 3;

AHRS *ahrs;

    bool autoBalanceXMode; //Auto balance stuff (not sure what it does)
    bool autoBalanceYMode;



  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

  //static const double kOffBalanceThresholdDegrees = 10.0f;  //More navx stuff, it's all red so have fun with that
//static const double kOnBalanceThresholdDegrees = 5.0f;

//using namespace std;
    



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
		int absolutePosition = armExtend.GetSensorCollection().GetPulseWidthPosition();
		/* use the low level API to set the quad encoder signal */
		armExtend.SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx,
				kTimeoutMs);
     //armExtend.SetSelectedSensorPosition(absolutePosition, int pidIdx = 1, int timeoutMs = 50); ?

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
  //ahrs->Reset(); <-crashes the code?

 // drivedistance = (drivedistance * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance2 = (drivedistance2 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance = (drivedistance3 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 


  SelectedAuto = frc::SmartDashboard::GetNumber("Selected Auto", 0);

  }

  void AutonomousPeriodic() override {
   
    if(SelectedAuto == 1){  //just driving backwards, with autobalance?

switch(switchOne) {
case 0:
  if(rightEncoder.GetPosition() < drivedistance){ 

        rightLeadmotor.Set(0.3);
      }
  if(leftEncoder.GetPosition() < drivedistance){

        rightLeadmotor.Set(0.3);
      }
case 1:
-Deadband(ahrs->GetPitch()*3, 0.5);

}


    };

  if(SelectedAuto == 2){   //Auto balance

  switch(switchTwo) {
  case 0:
    if(rightEncoder.GetPosition() < drivedistance2){ 

          rightLeadmotor.Set(0.3);
        }
    if(leftEncoder.GetPosition() < drivedistance2){

          rightLeadmotor.Set(0.3);
        }
case 1:
      -Deadband(ahrs->GetPitch()*3, 0.5);

case 2:
    if(rightEncoder.GetPosition() > drivedistance2){   

        rightLeadmotor.Set(0);
      }
    if(leftEncoder.GetPosition() > drivedistance2){   

        rightLeadmotor.Set(0);
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

  m_robotDrive.ArcadeDrive(Deadband(-m_stickDrive.GetY(), 0.05, 2), 0.5*Deadband(-m_stickDrive.GetZ(), 0.05, 2)); 

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

  
  if (m_stickOperator.GetRightBumperPressed()){
   intakemotorF.Set(0.5); // When pressed the intake turns on
  }

   if (m_stickOperator.GetRightBumperReleased()) {
   intakemotorF.Set(0); 
   } // When released the intake turns off

   if(m_stickOperator.GetRightBumperPressed()){
    intakemotorR.Set(-0.5);
   }

   if(m_stickOperator.GetRightBumperReleased()){
    intakemotorR.Set(0);
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

   /* get gamepad axis */
		double leftYstick = m_stickOperator.GetYButtonPressed();
		double motorOutput = armExtend.GetMotorOutputPercent();
		bool button1 = m_stickOperator.GetRawButton(1);
		bool button2 = m_stickOperator.GetRawButton(2);
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tpos:");
		_sb.append(std::to_string(armExtend.GetSelectedSensorPosition(kPIDLoopIdx)));
		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && button1) {
			/* Position mode - button just pressed */
			targetPositionRotations = leftYstick * 10.0 * 4096; /* 10 Rotations in either direction */
			armExtend.Set(ControlMode::Position, targetPositionRotations); /* 10 rotations in either direction */
		}
		/* on button2 just straight drive */
		if (button2) {
			/* Percent voltage mode */
			armExtend.Set(ControlMode::PercentOutput, leftYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (armExtend.GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(armExtend.GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetPositionRotations));
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();
		/* save button state for on press detect */
		_lastButton1 = button1;
  


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
