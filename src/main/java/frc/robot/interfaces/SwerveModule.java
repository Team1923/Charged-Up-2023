package frc.robot.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
/*
    * We begin by defining our motors.
    * Because we create SwerveModule objects
    * in the SwerveSubsystem, there is no need
    * to instantiate them here. We will do so in
    * the constructor.
    */

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    /*
    * Next we define our PID Controller for turning.
    We use the WPILib PIDController
    */

    private final PIDController turningPIDController;

    /*
    * Next, we create our Absolute Encoder.
    * We have 2 variables that go along with it.
    * The reversed variable measures if we need to 
    * change the diretionality of its reading. 
    * The offset is just a measure of the intial
    * reading of the encoder.
    */

    private final WPI_CANCoder canCoder;;
    private final double canCoderID;
    private final boolean canCoderReversed;
    private final double canCoderOffsetRad;

    /*
    * We then define some more variables to determine
        if we need to flip the direction of any of our motors
    */

    private final boolean driveMotorReversed;
    private final boolean turningMotorReversed;
    
    /*
        * Next, we write our constructor to create our SwerveModule object
        */

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        /*
            * We begin by instantiating everything necessary
            * for the absolute encoder
            */
        canCoder = new WPI_CANCoder(absoluteEncoderID, "Default Name");
        this.canCoderID = absoluteEncoderID;
        this.canCoderOffsetRad = absoluteEncoderOffset;
        this.canCoderReversed = absoluteEncoderReversed;
        canCoder.configFactoryDefault();
        if(canCoderReversed){
            canCoder.configSensorDirection(absoluteEncoderReversed);
        }

        canCoder.configMagnetOffset(absoluteEncoderOffset);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);


        /*
        * Motor-specific instantiations
        */
        driveMotor = new WPI_TalonFX(driveMotorId, "Default Name"); //assuming its on a CAN Bus
        turningMotor = new WPI_TalonFX(turningMotorId, "Default Name"); //assuming its on a CAN Bus
        this.driveMotorReversed = driveMotorReversed;
        this.turningMotorReversed = turningMotorReversed;
        configDriveMotor();
        configTurningMotor();

        /*
        * Finally, we intantiate the PID Controller. 
        * We will most definitely need to tune it for 
        * our new Swerve Modules
        */

        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, ModuleConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        /*
        * Once the module is instantiated,
        we reset our encoders.
        */
        resetEncoders();
        
    }

    /*
        * Below we have methods to get us all the information
        * we could want from our motors.
        */
    public double getDrivePositionMeters() {
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderTicks2Meter;
    }

    public double getTurningPositionRads() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kturningEncoderTicks2Rad;
    }

    public double getDriveVelocityMPS() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderTicks2MeterPerSec;
    }

    public double getTurningVelocityRPS() {
        return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderTicks2RadPerSec;
    }

    public double getTurningTicks() {
        return turningMotor.getSelectedSensorPosition();
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(canCoder.getPosition());
    }


    /*
        * WPILib has specific SwerveModule objects
        * that we need to return. It's pretty much
        * just taking the information from our
        * motor and making WPILib specific
        * objects out of them.
        */

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), new Rotation2d(getTurningPositionRads()));
    } 

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(-getDriveVelocityMPS(), new Rotation2d(getTurningPositionRads()));
    } 

    public void resetState() {
        SwerveModuleState reset_state = new SwerveModuleState(0, new Rotation2d(0));
        reset_state = SwerveModuleState.optimize(reset_state, getSwerveModuleState().angle);
        driveMotor.set(ControlMode.PercentOutput, (reset_state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPositionRads(), reset_state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    /*
        * Finally, we have a method that will
        * set the state of our SwerveModule. 
        * This is continuously updated.
        */

    public void setDesiredState(SwerveModuleState state) {
        /*If there is no substantial change in velocity from 
        current state to new state, do not change anything.
        This just negates the fact that the motors are 
        going to return to zero if you let go of the stick, 
        which is very annoying when driving
        */

        if(Math.abs(state.speedMetersPerSecond) < .001) {
                stop();
                return;
        }

        state = SwerveModuleState.optimize(state, getSwerveModuleState().angle);
        System.out.println(state.speedMetersPerSecond);

        driveMotor.set(ControlMode.PercentOutput, (state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPositionRads(), state.angle.getRadians()));

        // Recommended debug printout for swerve state
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "]", state.toString());
    }

    public void turningMotorPercentOut(){
        turningMotor.set(ControlMode.PercentOutput, 0.1);
    }

    public double getPIDError(){
        return turningPIDController.getPositionError();
    }

    // Configs Drive Motor
    public void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        if(this.driveMotorReversed) {
            driveMotor.setInverted(InvertType.InvertMotorOutput);
        } else {
            driveMotor.setInverted(InvertType.None);
        }
    }

    // Configs Turning Motor
    public void configTurningMotor() {
        turningMotor.configFactoryDefault();
        turningMotor.setNeutralMode(NeutralMode.Brake);
        if(this.turningMotorReversed) {
            turningMotor.setInverted(InvertType.InvertMotorOutput);
        } else {
            turningMotor.setInverted(InvertType.None);
        }   
    }

    public void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    }

    public void resetDriveEncoder() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public void resetTurningEncoder() {
        double absolute = getAbsoluteEncoderRad();
        double ticks_from_radians = ((absolute / (2 * Math.PI)) / ModuleConstants.kTurningGearRatio) * ModuleConstants.kTicksPerRotation;
        try {
            Thread.sleep(400);
            turningMotor.setSelectedSensorPosition(ticks_from_radians);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }  

}
