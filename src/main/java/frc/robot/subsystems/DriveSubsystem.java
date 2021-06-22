// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants.PhysicalRobotConstants;
import frc.robot.Constants.Ports;

public class DriveSubsystem extends SubsystemBase {
  WPI_TalonSRX leftFrontMaster = new WPI_TalonSRX(Ports.frontLeftID);
  WPI_TalonSRX rightFrontMaster = new WPI_TalonSRX(Ports.frontRightID);
  WPI_TalonSRX leftBackFollower = new WPI_TalonSRX(Ports.backLeftID);
  WPI_TalonSRX rightBackFollower = new WPI_TalonSRX(Ports.backRightID);

  SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFrontMaster, leftBackFollower);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFrontMaster, rightBackFollower);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

  AnalogGyro gyro = new AnalogGyro(1); //use AHRS in real life
  

 
  

  private final DifferentialDriveOdometry odometry;

  //SIMULATION OBJECTS
  TalonSRXSimCollection leftFrontMasterSim = new TalonSRXSimCollection(leftFrontMaster);
  TalonSRXSimCollection rightFrontMasterSim = new TalonSRXSimCollection(rightFrontMaster);

  AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

   //Simulation model of the drivetrain
   DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),        //2 CIMS on each side of the drivetrain.
    PhysicalRobotConstants.kGearRatio,               //Standard AndyMark Gearing reduction.
    2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
    26.5,                     //Mass of the robot is 26.5 kg.
    Units.inchesToMeters(PhysicalRobotConstants.kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
    PhysicalRobotConstants.kTrackWidthMeters,                    //Distance between wheels is _ meters.
    
    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Comment out this line if measurement noise is undesirable (replace with null)
  );

  Field2d field = new Field2d();
  //END OF SIMULATION OBJECTS


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetSensorCounts();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    SmartDashboard.putData("SimField", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getRotation2d(), 
      getLeftDistanceMeters(), 
      getRightDistanceMeters()
    );

    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to use
    // the output voltage, NOT the percent output.
    m_driveSim.setInputs(leftFrontMaster.getMotorOutputVoltage(),
                         rightFrontMaster.getMotorOutputVoltage()); //Right side is inverted, so forward is negative voltage

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftFrontMasterSim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                    m_driveSim.getLeftPositionMeters()));
    leftFrontMasterSim.setQuadratureVelocity(
                    velocityToNativeUnits(
                    m_driveSim.getLeftVelocityMetersPerSecond()));
    rightFrontMasterSim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                    m_driveSim.getRightPositionMeters()));
    rightFrontMasterSim.setQuadratureVelocity(
                    velocityToNativeUnits(
                    m_driveSim.getRightVelocityMetersPerSecond()));
    gyroSim.setAngle(-m_driveSim.getHeading().getDegrees()); 

    //Update other inputs to Talons
    leftFrontMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightFrontMasterSim.setBusVoltage(RobotController.getBatteryVoltage());

  }


  public void drive(double speed, double rotation, boolean squareInputs) {
    differentialDrive.arcadeDrive(speed, rotation, squareInputs);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts); //might need to negate
    differentialDrive.feed();
  }

  public void stop() {
    leftGroup.set(0);
    rightGroup.set(0);
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getLeftDistanceMeters() {
    return nativeUnitsToDistanceMeters(leftFrontMaster.getSelectedSensorPosition());
  }

  public double getRightDistanceMeters() {
    return nativeUnitsToDistanceMeters(rightFrontMaster.getSelectedSensorPosition());
  }

  public double getLeftVelocity() {
    return nativeUnitsToMetersPerSec(leftFrontMaster.getSelectedSensorVelocity());
  }
  public double getRightVelocity() {
    return nativeUnitsToMetersPerSec(rightFrontMaster.getSelectedSensorVelocity());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    //Units shOulD be in meters per second
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public void resetSensorCounts() {
    leftFrontMaster.setSelectedSensorPosition(0);
    rightFrontMaster.setSelectedSensorPosition(0);
    leftBackFollower.setSelectedSensorPosition(0);
    rightBackFollower.setSelectedSensorPosition(0);
    
    leftFrontMaster.set(0);
    rightFrontMaster.set(0);
    leftBackFollower.set(0);
    rightBackFollower.set(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetSensorCounts();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void resetHeading() {
    gyro.reset();
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }



  //UNIT CONVERSION METHODS
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / PhysicalRobotConstants.kFalconCPR;
    double wheelRotations = motorRotations / PhysicalRobotConstants.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(PhysicalRobotConstants.kWheelRadiusInches));
    return positionMeters;
  }

  private double nativeUnitsToMetersPerSec(double sensorCountsPerDecisecond) {
    double sensorCountsPerSecond = sensorCountsPerDecisecond * 10;
    double motorRotationsPerSecond = sensorCountsPerSecond / PhysicalRobotConstants.kFalconCPR; 
    double wheelRotationsPerSecond = motorRotationsPerSecond / PhysicalRobotConstants.kGearRatio; 
    double metersPerSecond = wheelRotationsPerSecond * (PhysicalRobotConstants.kWheelDiameterMeters * Math.PI);
    return metersPerSecond;

  }

  public int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(PhysicalRobotConstants.kWheelRadiusInches));
    double motorRotations = wheelRotations * PhysicalRobotConstants.kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * PhysicalRobotConstants.kFalconCPR);
    return sensorCounts;
  }

  public int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(PhysicalRobotConstants.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * PhysicalRobotConstants.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * PhysicalRobotConstants.kFalconCPR);
    return sensorCountsPer100ms;
  }
}
