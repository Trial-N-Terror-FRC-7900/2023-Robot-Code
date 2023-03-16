#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
//#include <photonlib/PhotonUtils.h>
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
#include <frc/Timer.h>
#include <frc/Encoder.h>

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
rev::CANSparkMax armRotate2{10, rev::CANSparkMax::MotorType::kBrushless};

//frc::Compressor phCompressor{15, frc::PneumaticsModuleType::REVPH};

//frc::DoubleSolenoid intakeS{15, frc::PneumaticsModuleType::REVPH, 2, 3};
//frc::DoubleSolenoid intakeN{15, frc::PneumaticsModuleType::REVPH, 4, 5};


CANdle candle{16};
//void candlePurple(){
        //candle.SetLEDs(75,0,130);}
//void candleGreen(){
        //candle.SetLEDs(0, 119, 20);}
//Animation(1, 0.5, 64, 0);

double scale = 250, offset = -25;
//frc::AnalogPotentiometer pressureTransducer{1, scale, offset};

//WPI_VictorSPX intakemotorF{9};
//WPI_VictorSPX intakemotorR{10}; 
WPI_VictorSPX handF{13};
WPI_VictorSPX handR{14}; 
WPI_TalonFX armExtend{6};

frc::Timer timer;

 /*
 * Change these parameters to match your setup
 */

/** 
* An alternate encoder object is constructed using the GetAlternateEncoder() 
* method on an existing CANSparkMax object. If using a REV Through Bore 
* Encoder, the type should be set to quadrature and the counts per 
* revolution set to 8192
*/

double armRGoal = 0;
double armEGoal = 0;


std::string _sb;
int kPIDLoopIdx;
bool kTimeoutMs;
int targetPositionRotations;

int _loops = 0;
bool _lastButton1 = false;

frc::DifferentialDrive m_robotDrive{rightLeadmotor, leftLeadmotor};
frc::Joystick m_stickDrive{0};
frc::XboxController m_stickOperator{1};

rev::SparkMaxAbsoluteEncoder armREncoder = armRotate.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
rev::SparkMaxPIDController armPID = armRotate.GetPIDController();

//rev::SparkMaxLimitSwitch forwardLimit = armRotate.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
//rev::SparkMaxLimitSwitch reverseLimit = armRotate2.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);

rev::SparkMaxRelativeEncoder rightEncoder = rightLeadmotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);
rev::SparkMaxRelativeEncoder leftEncoder = leftLeadmotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);


double drivedistance = 10; //120 inches i beleive
double drivedistance2 = 6; //72 inches
double drivedistance3 = -12; //if we are in the far RIGHT of BLUE
double drivedistance4 = 0.5; //pushing block/cone/whatever fowrard MAYBEEEEEEEEEEE
double SelectedAuto = 0; //selected auto


int autoStep = 0;
int loopCount = 0;
double boost = 0;

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
    //intakemotorF.ConfigFactoryDefault();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightLeadmotor.SetInverted(true);


    //PID start
    armPID.SetP(kP);
    armPID.SetI(kI);
    armPID.SetD(kD);
    armPID.SetIZone(kIz);
    armPID.SetFF(kFF);
    armPID.SetOutputRange(kMinOutput, kMaxOutput);
    armPID.SetFeedbackDevice(armREncoder);

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
    //forwardLimit.EnableLimitSwitch(false);
    //reverseLimit.EnableLimitSwitch(false);
    //frc::SmartDashboard::PutBoolean("Forward Limit Enabled", forwardLimit.IsLimitSwitchEnabled());
    //frc::SmartDashboard::PutBoolean("Reverse Limit Enabled", reverseLimit.IsLimitSwitchEnabled());

    //phCompressor.SetClosedLoopControl(false);

    //bool enabled = phCompressor.Enabled();
    //bool pressureSwitch = phCompressor.GetPressureSwitchValue();
    //double current = phCompressor.GetCurrent();
    
    m_robotDrive.SetDeadband(0);
    rightFollowmotor.Follow(rightLeadmotor);
    leftFollowmotor.Follow(leftLeadmotor);

    armRotate2.Follow(armRotate, true);
    armRotate.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armRotate2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armRotate.SetSmartCurrentLimit(20);
    armRotate2.SetSmartCurrentLimit(20);
    armRotate.SetInverted(true);

    armRotate.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 2);
    armRotate.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 252);

    armREncoder.SetPositionConversionFactor(360); // converts to degrees instead of rotations
    //armREncoder.GetInverted();

    armExtend.SetNeutralMode(NeutralMode::Brake);

    


    //intakeS.Set(frc::DoubleSolenoid::Value::kOff); //These are supposed to be the different levels it goes or something.
    //intakeS.Set(frc::DoubleSolenoid::Value::kForward);
    //intakeS.Set(frc::DoubleSolenoid::Value::kReverse);

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
		armExtend.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,kTimeoutMs);
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
  
  RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  candle.Animate(*rainbowAnim);

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
  autoStep = 0;
  rightEncoder.SetPositionConversionFactor(0.16095446232);
  leftEncoder.SetPositionConversionFactor(0.16095446232);
  rightEncoder.SetPosition(0);
  leftEncoder.SetPosition(0);
  ahrs->Reset();
  //ahrs->Reset(); <-crashes the code?


 // drivedistance = (drivedistance * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance2 = (drivedistance2 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 
 // drivedistance = (drivedistance3 * 12)/(6 * 3.141592635) * (34/18) * (62/12) * (42); 


  SelectedAuto = frc::SmartDashboard::GetNumber("Selected Auto", 0);
  if(SelectedAuto == 0){
    rightLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }
  else{
    rightLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    leftLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    leftFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  }
  //RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  //candle.Animate(*rainbowAnim);
}

void AutonomousPeriodic() override {
 //  RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 64);
 // candle.Animate(*rainbowAnim);
  frc::SmartDashboard::PutNumber("Nav X Yaw", ahrs->GetAngle());
  frc::SmartDashboard::PutNumber("Nav X Roll", ahrs->GetRoll());
  frc::SmartDashboard::PutNumber("Nav X Pitch", ahrs->GetPitch());

  if(SelectedAuto == 1){  //just driving 10ft, with autobalance?

    frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
    if(rightEncoder.GetPosition() < 10.0){ 
      m_robotDrive.ArcadeDrive(0.4, 0); 
    }
    else{
      m_robotDrive.ArcadeDrive(0, 0); 
    }

  }

  if(SelectedAuto == 2){   //drive different distance, about 6ft, to get onto platform?
    frc::SmartDashboard::PutNumber("auto step", autoStep);
    if(autoStep == 0){
        frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        frc::SmartDashboard::PutString("AutoPart", "Part1");
        if(rightEncoder.GetPosition() < 18.0){ 
          m_robotDrive.ArcadeDrive(0.6, 0); 
        }
        else{
          frc::SmartDashboard::PutString("AutoPart", "Part2");
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep = 1;
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
          timer.Start();
        }
    }

    if(autoStep == 1){
      if (timer.Get() < 2_s)
      {
       m_robotDrive.ArcadeDrive(0, 0);
      }
      else{
        timer.Stop();
        autoStep = 2;
      }
    }

    if(autoStep == 2){
      frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
      if(rightEncoder.GetPosition() > -8.5){ 
        m_robotDrive.ArcadeDrive(0.6, 0); 
      }
      else{
        m_robotDrive.ArcadeDrive(0, 0); 
        autoStep = 3;
        rightEncoder.SetPosition(0); 
        leftEncoder.SetPosition(0);
      }
    }
    if(autoStep == 3){
          frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
      //if(fabs(rightEncoder.GetPosition()) < 2.0){
        m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
      //}
    }
  }

  if(SelectedAuto == 3){
          frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
      //if(fabs(rightEncoder.GetPosition()) < 2.0){
        m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
 // }
}


  if(SelectedAuto == 4){ //8ft, turn left, 3 ft, turn left, 3ft
      frc::SmartDashboard::PutNumber("auto step", autoStep);
    if(autoStep == 0){
        frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        if(rightEncoder.GetPosition() < 12){ 
          m_robotDrive.ArcadeDrive(0.8, 0); 
        }
        else{
          ahrs->Reset();
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep = 1;
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
      }
    if(autoStep == 1){
        double gyro = ahrs->GetYaw();
        double	z;
        double turn = -90;

        if(loopCount == 0){
          timer.Start();
        }

        if(timer.Get() >= 0.5_s){
          boost = boost + 0.003;
        }

        z = ((gyro - turn) * 0.008+boost);
        if(fabs(gyro-turn) < 1){
          ahrs->Reset();  
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
          autoStep++;
          boost = 0;
          loopCount = 0;
          timer.Stop();
          timer.Reset();
          m_robotDrive.ArcadeDrive(0, 0);
          timer.Start();
        }
        else{
          loopCount++;
          m_robotDrive.ArcadeDrive(0, z);
        }
    }
    if(autoStep == 2){ // Needs a Delay for some reason otherwise it wont drive forward properly
      if(timer.Get() > 0.5_s){
        autoStep++;
      }
    }
    if(autoStep == 3){
        frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        frc::SmartDashboard::PutString("AutoPart", "Part1");
        if(rightEncoder.GetPosition() > -3.5){ 
          m_robotDrive.ArcadeDrive(-0.6, 0); 
        }
        else{
          frc::SmartDashboard::PutString("AutoPart", "Part2");
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep++;
          ahrs->Reset();
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
     }
    if(autoStep == 4){
        double gyro = ahrs->GetYaw();
        double	z;
        double turn = -90;

        if(loopCount == 0){
          timer.Start();
        }

        if(timer.Get() >= 0.5_s){
          boost = boost + 0.003;
        }

        z = ((gyro - turn) * 0.008+boost);
        if(fabs(gyro-turn) < 0.5){
          ahrs->Reset();  
          autoStep++;
          boost = 0;
          loopCount = 0;
          timer.Stop();
          timer.Reset();
          m_robotDrive.ArcadeDrive(0, 0);
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
        else{
          loopCount++;
          m_robotDrive.ArcadeDrive(0, z);
        }

    }
    if(autoStep == 5){
        frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        frc::SmartDashboard::PutString("AutoPart", "Part1");
        if(fabs(rightEncoder.GetPosition()) < 5.5){ 
          m_robotDrive.ArcadeDrive(0.6, 0); 
        }
        else{
          frc::SmartDashboard::PutString("AutoPart", "Part2");
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep = 6;
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
    }
    if(autoStep == 6){
          frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
        m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
    }
  }


  if(SelectedAuto == 5){ //8ft, turn right, 3 ft, turn right, 3ft
    frc::SmartDashboard::PutNumber("auto step", autoStep);
    if(autoStep == 0){
        frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        frc::SmartDashboard::PutString("AutoPart", "Part1");
        if(rightEncoder.GetPosition() < 12){ 
          m_robotDrive.ArcadeDrive(0.6, 0); 
        }
        else{
          frc::SmartDashboard::PutString("AutoPart", "Part2");
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep = 1;
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
      }
      if(autoStep == 1){
        double gyro = ahrs->GetYaw();
        double	z;
        double turn = -90;

        if(loopCount == 0){
          timer.Start();
        }

        if(timer.Get() >= 0.5_s){
          boost = boost + 0.003;
        }

        z = ((gyro - turn) * 0.008+boost);
        if(fabs(gyro-turn)< 1){
          ahrs->Reset();  
          autoStep++;
          boost = 0;
          loopCount = 0;
          timer.Stop();
          timer.Reset();
          m_robotDrive.ArcadeDrive(0, 0);
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
        else{
          loopCount++;
          m_robotDrive.ArcadeDrive(0, z);
        }
      }
      if(autoStep == 2){
          frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
          frc::SmartDashboard::PutString("AutoPart", "Part1");
          if(fabs(rightEncoder.GetPosition()) < 3.0){ 
            m_robotDrive.ArcadeDrive(0.6, 0); 
          }
          else{
            frc::SmartDashboard::PutString("AutoPart", "Part2");
            m_robotDrive.ArcadeDrive(0, 0); 
            ahrs->Reset();
            autoStep = 3;
            rightEncoder.SetPosition(0); 
            leftEncoder.SetPosition(0);
          }
      }
      if(autoStep == 3){
        double gyro = ahrs->GetYaw();
        double	z;
        double turn = -90;

        if(loopCount == 0){
          timer.Start();
        }

        if(timer.Get() >= 0.5_s){
          boost = boost + 0.003;
        }

        z = ((gyro - turn) * 0.008+boost);
        if(fabs(gyro-turn)< 1){
          ahrs->Reset();  
          autoStep++;
          boost = 0;
          loopCount = 0;
          timer.Stop();
          timer.Reset();
          m_robotDrive.ArcadeDrive(0, 0);
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
        }
        else{
          loopCount++;
          m_robotDrive.ArcadeDrive(0, z);
        } 
      }
      if(autoStep == 4){
          frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
          frc::SmartDashboard::PutString("AutoPart", "Part1");
          if(rightEncoder.GetPosition() < 4.5){ 
            m_robotDrive.ArcadeDrive(0.6, 0); 
          }
          else{
            frc::SmartDashboard::PutString("AutoPart", "Part2");
            m_robotDrive.ArcadeDrive(0, 0); 
            autoStep = 5;
            rightEncoder.SetPosition(0); 
            leftEncoder.SetPosition(0);
          }
      }
      if(autoStep == 5){
            frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
          m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
      }
    }

  if(SelectedAuto == 6){
    double gyro = ahrs->GetYaw();
    double	z = ((gyro - 90) * 0.012);
	  if(fabs(gyro-90)< 4){  
      autoStep++;
  	}

  	m_robotDrive.ArcadeDrive(0, z);
  	frc::SmartDashboard::PutNumber("Gyro", gyro);
  }
  
  if(SelectedAuto == 7){
    if(autoStep == 0){
      double gyro = ahrs->GetYaw();
      double	z;
      double turn = 90;

      if(loopCount == 0){
        timer.Start();
      }

      if(timer.Get() >= 1_s){
        boost = boost + 0.0005;
      }

      z = ((gyro - turn) * 0.008+boost);
      if(fabs(gyro-turn)< 0.5 && timer.Get() > 1_s){  
        autoStep++;
        boost = 0;
        loopCount = 0;
        timer.Stop();
        timer.Reset();
      }
      
      loopCount++;
      m_robotDrive.ArcadeDrive(0, z);
    }
    if(autoStep == 1){
      m_robotDrive.ArcadeDrive(0, 0);
    }

    

  }
  
  if(SelectedAuto == 8){ //This hypothetically place something and the move out of community ?
    if(autoStep == 0){

     armExtend.Set(ControlMode::Position, 5000);
     autoStep = 1;
    }
    if(autoStep == 1){

     handF.Set(0.3);
     autoStep = 2;
    }
    if(autoStep == 2){

     frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
      if(rightEncoder.GetPosition() < 12){ 
       m_robotDrive.ArcadeDrive(0.8, 0); 
      }
      else{
       ahrs->Reset();
       m_robotDrive.ArcadeDrive(0, 0);
       rightEncoder.SetPosition(0); 
       leftEncoder.SetPosition(0);
        }
    }
  }

}



  // Teleop Area
  void TeleopInit() override {

    

  }

  void TeleopPeriodic() override {
    //double StickX = Deadband(-m_stick.GetX(), 0.05, 2);
    //double StickY = Deadband(-m_stick.GetY(), 0.05, 2); 

    //frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
     if(m_stickDrive.GetRawButton(4)){ 
      //frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
      m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
    }
    else{
      if(m_stickDrive.GetRawButton(1)){
        m_robotDrive.ArcadeDrive(0.2*(Deadband(m_stickDrive.GetY(), 0.05, 2)), 0.5*Deadband(-m_stickDrive.GetZ(), 0.05, 2));
      }
      else{
        m_robotDrive.ArcadeDrive(Deadband(m_stickDrive.GetY(), 0.05, 2), 0.5*Deadband(-m_stickDrive.GetZ(), 0.05, 2)); 
      }
    }

    if(m_stickDrive.GetRawButton(3)){
      ahrs->Reset();
      ahrs->ZeroYaw();
      ahrs->GetBoardYawAxis();
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

    double armRotaion = armREncoder.GetPosition();
    frc::SmartDashboard::PutNumber("Arm Roatation Encoder", armRotaion);

    double armExtention = armExtend.GetSelectedSensorPosition();
    frc::SmartDashboard::PutNumber("Arm Extention Encoder", armExtention);

    /*
    if(fabs(armRotaion - armRGoal) > 30/360){ //roataion in rotaions lol not degree bUT I CAN DIVIDE
      if(armExtend.GetSelectedSensorPosition(0) > 2){
          armExtend.Set(ControlMode::Position, 0);
      }
      else{
        armPID.SetReference(armRGoal, rev::CANSparkMax::ControlType::kPosition); //roate :thumbs_up:
      }
    }
    else{
       armExtend.Set(ControlMode::Position, armEGoal);
    }
    */

		double leftYstick = m_stickOperator.GetLeftY();
	
		//press button and it'll go where you want it to go
		if (m_stickOperator.GetYButtonPressed()) {
      armRGoal = 0.03;
      armEGoal = 5000;
      armPID.SetReference(30, rev::CANSparkMax::ControlType::kPosition);
		}

    if (m_stickOperator.GetXButtonPressed()) {
      armRGoal = 0.02;
      armEGoal = 4500;
      armPID.SetReference(90, rev::CANSparkMax::ControlType::kPosition);
		}

    if (m_stickOperator.GetAButtonPressed()) {
      armRGoal = 0.01;
      armEGoal = 4000;
      armPID.SetReference(180, rev::CANSparkMax::ControlType::kPosition);
		}
		//override, add deadband for xbox controller 
		/*
    if (m_stickOperator.GetRightBumperPressed()) {
			armExtend.Set(ControlMode::PercentOutput, Deadband(leftYstick, 0.01));
		}
    */
    
    //coast mode
    if (m_stickDrive.GetRawButton(5)){
     rightLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
     rightFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
     leftLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
     leftFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }

    //break mode
    if (m_stickDrive.GetRawButton(6)){
     rightLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
     rightFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
     leftLeadmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
     leftFollowmotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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

    if((p != kP)) { armPID.SetP(p); kP = p; }
    if((i != kI)) { armPID.SetI(i); kI = i; }
    if((d != kD)) { armPID.SetD(d); kD = d; }
    if((iz != kIz)) { armPID.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { armPID.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      armPID.SetOutputRange(min, max);
    }
      kMinOutput = min; kMaxOutput = max;
  //end PID control
    
    //frc::SmartDashboard::PutNumber("SetPoint", rotations);
    //frc::SmartDashboard::PutNumber("ProcessVariable", armREncoder.GetPosition());

    //frc::SmartDashboard::PutBoolean("Forward Limit Switch", forwardLimit.Get());
    //frc::SmartDashboard::PutBoolean("Reverse Limit Switch", forwardLimit.Get());
    //end spark max limit switch

 //bool motionDetected = ahrs->IsMoving();  //More code from the only example of navx code I could find, I am going crazy rn
  //SmartDashboard::PutBoolean("MotionDetected", motionDetected);


}

void DisabledInit() override {

}

void DisabledPeriodic() override {
   frc::SmartDashboard::PutNumber("Nav X Yaw", ahrs->GetAngle());
   frc::SmartDashboard::PutNumber("Nav X Roll", ahrs->GetRoll());
   frc::SmartDashboard::PutNumber("Nav X Pitch", ahrs->GetPitch());

  double displayauto = frc::SmartDashboard::GetNumber("Selected Auto", 0);

   frc::SmartDashboard::PutNumber("Auto Number Received:", displayauto);

   double armRotaion = armREncoder.GetPosition();
   frc::SmartDashboard::PutNumber("Arm Roatation Encoder", armRotaion);

   double armExtention = armExtend.GetSelectedSensorPosition();
   frc::SmartDashboard::PutNumber("Arm Extention Encoder", armExtention);

}


 
};  

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(StickY, StickX);
  


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
