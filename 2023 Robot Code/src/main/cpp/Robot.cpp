#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
//#include <photonlib/PhotonUtils.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/PowerDistribution.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/Compressor.h>
#include <frc/XboxController.h>
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
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

frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};

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

double maxForwardExtension = 65; // inches
double maxRearExtension = 65; // inches

double maxVertialExtensionDown = 34; // inches
double maxVertialExtensionUp = ((6*12)+6)-maxVertialExtensionDown; //inches


double TopRightCornerAngle = atan2(maxVertialExtensionUp, maxForwardExtension) * (180/3.141592);
double BottomRightCornerAngle = atan2(-maxVertialExtensionDown, maxForwardExtension) * (180/3.141592);

double TopLeftCornerAngle = atan2(maxVertialExtensionUp, -maxRearExtension) * (180/3.141592);
double BottomLeftCornerAngle = atan2(-maxVertialExtensionDown, -maxRearExtension) * (180/3.141592);

double maxArmExtension = 0;

bool isFront = true;
bool isStowed = true;
bool holdPiece = false;

bool ArmZeroed = false;

std::string _sb;
int kPIDLoopIdx = 0;

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

RainbowAnimation *rainbowAnim = new RainbowAnimation(1, 0.5, 128);


double kP = 0.045, kI = 1e-5, kD = 0.07, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

  //static const double kOffBalanceThresholdDegrees = 10.0f;  //More navx stuff, it's all red so have fun with that
//static const double kOnBalanceThresholdDegrees = 5.0f


 public: 
  void RobotInit() override {
    // Factory Reset all Devices
    rightLeadmotor.RestoreFactoryDefaults();
    rightFollowmotor.RestoreFactoryDefaults();
    leftLeadmotor.RestoreFactoryDefaults();
    leftFollowmotor.RestoreFactoryDefaults();
    //intakemotorF.ConfigFactoryDefault();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    //armRotate.RestoreFactoryDefaults(); //NEED TO SETUP after encoder offset is in the code
    armRotate2.RestoreFactoryDefaults();

    armExtend.ConfigFactoryDefault();
    armExtend.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0);
    armExtend.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector, ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
    armExtend.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector, ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled);

    handF.ConfigFactoryDefault();
    handR.ConfigFactoryDefault();
    
    candle.ConfigFactoryDefault();
    PDH.ResetTotalEnergy();

    rightLeadmotor.SetInverted(true); // Inverts one side of the drive train


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
    //armREncoder.SetZeroOffset(15); //NEED TO SETUP

    armExtend.SetNeutralMode(NeutralMode::Brake);


    handF.SetNeutralMode(NeutralMode::Brake);
    handR.SetNeutralMode(NeutralMode::Brake);
    


    //intakeS.Set(frc::DoubleSolenoid::Value::kOff); //These are supposed to be the different levels it goes or something.
    //intakeS.Set(frc::DoubleSolenoid::Value::kForward);
    //intakeS.Set(frc::DoubleSolenoid::Value::kReverse);

    int _loops = 0;
	  bool _lastButton1 = false;
	  /** save the target position to servo to */
	  double targetPositionRotations;

    /**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		//int absolutePosition = armExtend.GetSensorCollection().GetIntegratedSensorAbsolutePosition();
		/* use the low level API to set the quad encoder signal */
		//armExtend.SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		//armExtend.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,kTimeoutMs);
		//armExtend.SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		armExtend.ConfigNominalOutputForward(0);
		armExtend.ConfigNominalOutputReverse(0);
		armExtend.ConfigPeakOutputForward(0.25);
		armExtend.ConfigPeakOutputReverse(-0.5);

		/* set closed loop gains in slot0 */
		armExtend.Config_kF(kPIDLoopIdx, 0.0);
		armExtend.Config_kP(kPIDLoopIdx, 1);
		armExtend.Config_kI(kPIDLoopIdx, 0);
		armExtend.Config_kD(kPIDLoopIdx, 0);
    armExtend.ConfigAllowableClosedloopError(0, 100);

    armExtend.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    armExtend.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    armExtend.ConfigMotionCruiseVelocity(15, 10);
    armExtend.ConfigMotionAcceleration(15, 10);

    CANdleConfiguration config;

    config.stripType = LEDStripType::RGB;

    config.brightnessScalar = 0.5; 

    candle.ConfigAllSettings(config);
  

    candle.Animate(*rainbowAnim);

    try{
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
    */
      ahrs = new AHRS(frc::SPI::Port::kMXP);
    }
    catch (std::exception &ex){
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char * p_err_msg = err_msg.c_str();
    }

    frc::DataLogManager::Start(); // Start Logging the Robot Data
    // Record both DS control and joystick data
    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  };

  void LogData(){
    double displayauto = frc::SmartDashboard::GetNumber("Selected Auto", 0);
    frc::SmartDashboard::PutNumber("Auto Number Received:", displayauto);

    frc::SmartDashboard::PutNumber("Nav X Yaw", ahrs->GetAngle());
    frc::SmartDashboard::PutNumber("Nav X Roll", ahrs->GetRoll());
    frc::SmartDashboard::PutNumber("Nav X Pitch", ahrs->GetPitch());
    frc::SmartDashboard::PutNumber("Nav X Raw Accel X", ahrs->GetRawAccelX());
    frc::SmartDashboard::PutNumber("Nav X Raw Accel Y", ahrs->GetRawAccelY());
    frc::SmartDashboard::PutNumber("Nav X Raw Accel Z", ahrs->GetRawAccelZ());
    frc::SmartDashboard::PutNumber("Nav X World Linear Accel X", ahrs->GetWorldLinearAccelX());
    frc::SmartDashboard::PutNumber("Nav X World Linear Accel Y", ahrs->GetWorldLinearAccelY());
    frc::SmartDashboard::PutNumber("Nav X World Linear Accel Z", ahrs->GetWorldLinearAccelZ());
    
    frc::SmartDashboard::PutNumber("PDH Temperature (C)", PDH.GetTemperature());
    frc::SmartDashboard::PutNumber("PDH Total Energy", PDH.GetTotalEnergy());
    frc::SmartDashboard::PutNumber("PDH Total Current", PDH.GetTotalCurrent());
    frc::SmartDashboard::PutNumber("PDH Battery Voltage", PDH.GetVoltage());

    frc::SmartDashboard::PutNumber("Right Lead Drive Motor Bus Voltage", rightLeadmotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Right Lead Drive Motor Output Current", rightLeadmotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Right Lead Drive Motor Temp", rightLeadmotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Right Lead Drive Applied Output", rightLeadmotor.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Bus Voltage", rightFollowmotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Output Current", rightFollowmotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Temp", rightFollowmotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Left Follow Drive Applied Output", rightFollowmotor.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Left Lead Drive Motor Bus Voltage", leftLeadmotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Left Lead Drive Motor Output Current", leftLeadmotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Left Lead Drive Motor Temp", leftLeadmotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Left Lead Drive Applied Output", leftLeadmotor.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Bus Voltage", leftFollowmotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Output Current", leftFollowmotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Left Follow Drive Motor Temp", leftFollowmotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Left Follow Drive Applied Output", leftFollowmotor.GetAppliedOutput());

    double armRotaion = armREncoder.GetPosition();
    frc::SmartDashboard::PutNumber("Arm Rotation Encoder", armRotaion);
    frc::SmartDashboard::PutNumber("Arm Rotate Applied Output", armRotate.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Arm Rotate Bus Voltage", armRotate.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm Rotate Output Current", armRotate.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Rotate Temp", armRotate.GetMotorTemperature());

    frc::SmartDashboard::PutNumber("Arm Rotate2 Applied Output", armRotate2.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Arm Rotate2 Bus Voltage", armRotate2.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm Rotate2 Output Current", armRotate2.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Rotate2 Temp", armRotate2.GetMotorTemperature());

    frc::SmartDashboard::PutNumber("HandF Bus Voltage", handF.GetBusVoltage());
    //frc::SmartDashboard::PutNumber("HandF Temp", handF.GetTemperature());
    // Victor SPX does not support reading current
    frc::SmartDashboard::PutNumber("HandF Current", PDH.GetCurrent(16)); // FIX CHANNEL OF PDH 
    frc::SmartDashboard::PutNumber("HandF Applied Output", handF.GetMotorOutputVoltage());

    frc::SmartDashboard::PutNumber("HandR Bus Voltage", handR.GetBusVoltage());
    //frc::SmartDashboard::PutNumber("HandR Temp", handR.GetTemperature());
    // Victor SPX does not support reading current
    frc::SmartDashboard::PutNumber("HandR Current", PDH.GetCurrent(15)); // FIX CHANNEL OF PDH
    frc::SmartDashboard::PutNumber("HandR Applied Output", handR.GetMotorOutputVoltage());

    double armExtention = armExtend.GetSelectedSensorPosition();
    frc::SmartDashboard::PutNumber("Arm Extention Encoder", armExtention);
    frc::SmartDashboard::PutNumber("Arm Extention Bus Voltage", armExtend.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm Extention Output Voltage", armExtend.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Arm Extention Temp", armExtend.GetTemperature());
    frc::SmartDashboard::PutNumber("Arm Extention Output Current", armExtend.GetStatorCurrent());

    frc::SmartDashboard::PutNumber("CANdle Curret", candle.GetCurrent());
    frc::SmartDashboard::PutNumber("CANdle Temp", candle.GetTemperature());
    frc::SmartDashboard::PutNumber("CANdle Bus Voltage", candle.GetBusVoltage());
  }

  // Auto Area
  void AutonomousInit() override {
  autoStep = 0;
  rightEncoder.SetPositionConversionFactor(0.16095446232);
  leftEncoder.SetPositionConversionFactor(0.16095446232);
  rightEncoder.SetPosition(0);
  leftEncoder.SetPosition(0);
  ahrs->Reset();


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
  if(ArmZeroed == false){
    armExtend.Set(ControlMode::PercentOutput, -0.05);
  }
  else{
    armExtend.Set(ControlMode::MotionMagic, 0);
  }
}

void AutonomousPeriodic() override {

  if(ArmZeroed == false){
    if(armExtend.GetControlMode() == ControlMode::PercentOutput){
      if(armExtend.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
        armExtend.SetSelectedSensorPosition(0,0,0);
        ArmZeroed = true;
        armExtend.Set(ControlMode::Position, 0);

      }
    }
  }

  if(armExtend.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
    armExtend.SetSelectedSensorPosition(0,0,0);
    ArmZeroed = true;
  }

  LogData();

  if(SelectedAuto == 1){  //just driving 10ft

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

  if(SelectedAuto == 9){ //THROW and balance
    if(autoStep == 0){
      handF.Set(-0.3);
      handR.Set(-0.3);
      timer.Start();
    }
    else{
      autoStep == 1;
    }
    if(autoStep == 1){
      if(timer.Get() >= 1_s){
        handF.Set(0);
        handR.Set(0);
      }
      else{
        autoStep == 2;
      }
    }
      if(autoStep == 2){
         frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
        frc::SmartDashboard::PutString("AutoPart", "Part1");
        if(rightEncoder.GetPosition() < 15.0){ 
          m_robotDrive.ArcadeDrive(0.6, 0); 
        }
        else{
          frc::SmartDashboard::PutString("AutoPart", "Part2");
          m_robotDrive.ArcadeDrive(0, 0); 
          autoStep = 3;
          rightEncoder.SetPosition(0); 
          leftEncoder.SetPosition(0);
          timer.Start();
        }
      }
      if(autoStep == 3){
        if (timer.Get() < 2_s)
      {
       m_robotDrive.ArcadeDrive(0, 0);
      }
      else{
        timer.Stop();
        autoStep ++;
      }
      }
      if(autoStep == 4){
              frc::SmartDashboard::PutNumber("EncoderPos", rightEncoder.GetPosition());
      if(rightEncoder.GetPosition() > -8.5){ 
        m_robotDrive.ArcadeDrive(0.6, 0); 
      }
      else{
        m_robotDrive.ArcadeDrive(0, 0); 
        autoStep ++;
        rightEncoder.SetPosition(0); 
        leftEncoder.SetPosition(0);
      }
      }
      if(autoStep == 5){
        frc::SmartDashboard::PutNumber("Robot Pitch", ahrs->GetRoll());
        m_robotDrive.ArcadeDrive(-0.025*(ahrs->GetRoll()), 0);
      }
  }

}



  // Teleop Area
  void TeleopInit() override {
      if(ArmZeroed == false){
        armExtend.Set(ControlMode::PercentOutput, -0.3);
      }
      else{
        armExtend.Set(ControlMode::Position, 0);
      }
  }

  void TeleopPeriodic() override { 
    LogData();

    if(m_stickDrive.GetRawButton(4)){ 
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

  double RightOperatorTrigger = Deadband(m_stickOperator.GetRightTriggerAxis(), 0.005);
  double LeftOperatorTrigger = Deadband(m_stickOperator.GetLeftTriggerAxis(), 0.005);

  if(fabs(RightOperatorTrigger) > 0.005){
    handF.Set(RightOperatorTrigger);
    handR.Set(RightOperatorTrigger);
  }
  else if(fabs(LeftOperatorTrigger) > 0.005){
    handF.Set(-LeftOperatorTrigger);
    handR.Set(-LeftOperatorTrigger);
  }
  else{ // Hold the piece by applying 2 volts
    frc::SmartDashboard::PutBoolean("Hold", holdPiece);
    if(holdPiece == true){ // Holds the piece at a voltage compinsated 2 Volts
      handF.Set(0.2);
      handR.Set(0.2);
    }
    else{
      handF.Set(0);
      handR.Set(0);
    }
  }

  if(m_stickOperator.GetBButtonPressed()){
    holdPiece = !holdPiece;
  }



   if(m_stickDrive.GetRawButton(7)){
      candle.ClearAnimation(0);
   }
   if(m_stickDrive.GetRawButtonPressed(8)){
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
    //frc::SmartDashboard::PutNumber("Arm Roatation Encoder", armRotaion);

    double armExtention = armExtend.GetSelectedSensorPosition();
    //frc::SmartDashboard::PutNumber("Arm Extention Encoder", armExtention);

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
    frc::SmartDashboard::PutNumber("ArmExtend Reverse Limit Switch", armExtend.GetSensorCollection().IsRevLimitSwitchClosed());
    if(ArmZeroed == false){
      if(armExtend.GetControlMode() == ControlMode::PercentOutput){
        if(armExtend.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
          armExtend.SetSelectedSensorPosition(0,0,0);
          ArmZeroed = true;
          armExtend.Set(ControlMode::Position, 0);

        }
      }
    }
    if(armExtend.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
      armExtend.SetSelectedSensorPosition(0,0,0);
      ArmZeroed = true;
    }

    double armEposIN = armExtend.GetSelectedSensorPosition() * 10.61 * 3.14159 * (1+(0.063*floor(armExtend.GetSelectedSensorPosition()/2048)));

    double MaxArmLengthXFront = maxForwardExtension/(cos((3.141592/180)*armRotaion));
    double MaxArmLengthXRear = maxRearExtension/(cos((3.141592/180)*armRotaion));
    double MaxArmLengthYTop = maxVertialExtensionUp/(sin((3.141592/180)*armRotaion));
    double MaxArmLengthYBottom = maxVertialExtensionDown/(sin((3.141592/180)*armRotaion));

    if(armRotaion < BottomRightCornerAngle){
        maxArmExtension = MaxArmLengthYBottom; //# this should prevent it from hitting the floor or the chassis but not both. 
    }
    else if(armRotaion < TopRightCornerAngle){
        maxArmExtension = MaxArmLengthXRear; //# this should prevent it from extending more than 4 feet past the end of the robot
    }
    else if(armRotaion < TopLeftCornerAngle){
        maxArmExtension = MaxArmLengthYTop; //# this should prevent it from exceeding max height
    }
    else if(armRotaion < BottomLeftCornerAngle){
        maxArmExtension = MaxArmLengthXFront; //# this should prevent it from extending more than 4 feet past the front of the robot
    }
    else{
        maxArmExtension = MaxArmLengthYBottom; //# this should prevent it from hitting the floor or the chassis but not both. 
    }

		//press button and it'll go where you want it to go
		if (m_stickOperator.GetYButtonPressed()) {
      armRGoal = 30;
      armEGoal = 5000;
      //armPID.SetReference(30, rev::CANSparkMax::ControlType::kPosition);
      armPID.SetReference(60, rev::CANSparkMax::ControlType::kPosition);
		}

    if (m_stickOperator.GetXButtonPressed()) {
      armRGoal = 60;
      armEGoal = 4500;
      //armPID.SetReference(60, rev::CANSparkMax::ControlType::kPosition);
      armExtend.Set(ControlMode::Position, 50000);
		}

    if (m_stickOperator.GetAButtonPressed()) {
      armRGoal = 90;
      armEGoal = 4000;
      armExtend.Set(ControlMode::Position, 100000);
		}
    
    if (m_stickOperator.GetPOV(0)) {
      armRGoal = -30;
      armEGoal = -5000;
      armPID.SetReference(30, rev::CANSparkMax::ControlType::kPosition);
		}

    if (m_stickOperator.GetPOV(180)) {
      armRGoal = -60;
      armEGoal = -4500;
      armPID.SetReference(60, rev::CANSparkMax::ControlType::kPosition);
		}
    
    armExtend.Set(ControlMode::Position, armEGoal+(10*Deadband(-m_stickOperator.GetLeftY(), 0.005)));

    double countsforgoal = armEGoal/(3.14159*(2-0.05*(floor(armExtend.GetSelectedSensorPosition()/(10.61*2048)))));

    double countsforMax = maxArmExtension/(3.14159*(2-0.05*(floor(armExtend.GetSelectedSensorPosition()/(10.61*2048)))));

    
    if(fabs(armRotaion - armRGoal) < 20){
        // dont need to do a full retract for small moves
        if(armEGoal > maxArmExtension){ //# if requested position is greater than the limits
            armExtend.Set(ControlMode::Position, maxArmExtension); // just go to max extension for rotation
        }
        else{
            armExtend.Set(ControlMode::Position, armEGoal); // otherwise go to requested position
        }
        armPID.SetReference(armRGoal, rev::CANSparkMax::ControlType::kPosition);
    }
    else{
        armExtend.Set(ControlMode::Position, 0); //# fully retract arm to allow full rotation
        if(armEposIN < 12){ //# if retracted to less than 1 foot rotate
            armPID.SetReference(armRGoal, rev::CANSparkMax::ControlType::kPosition);
        }
    }
    
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



}


 
};  
  


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
