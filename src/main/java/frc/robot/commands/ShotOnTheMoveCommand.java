package frc.robot.commands;
import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.HubLookUpTable.ShootingParameters;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.shooter.shotmap.ShooterCalibration;
import frc.robot.subsystems.shooter.shotmap.ShotMapOutput;
import frc.robot.subsystems.shooter.shotmap.ShotMaps;
import frc.robot.subsystems.shooter.shotmap.SotmSolution;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class ShotOnTheMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private Limelight m_limelight;
  private HubLookUpTable m_HubLookUpTable = new HubLookUpTable();
  private boolean m_emptyHopper=false;
  private double m_lastHoodPos=0;
  private double m_lastDistance=60;//24; // default distance to use so will take mid shot if limelight is not responding
  private double m_timeLimit=0;
  private double overrideDistance=0;
  private boolean overrideDistanceFlag=false;
  //private double m_heldTurretAngle=0; // Angle to hold the turret at during shooting
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,NOFUEL,EMPTYHOPPER,SHOOTMORE,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;
  private double fuelcheckStartTime;
  private final double kfuelcheckWait=2;
  private int m_flipCount=0;
  private int m_flipCountLimit=0;
  private final int kflipCountMax=6;//35;
  private final int kHopperPullLimit=14;
  private CommandSwerveDrivetrain m_swerve;
  private int m_HopperPulls=0;
  //private boolean NoLimeLightMode=0;
  private double m_flipSpeed=0;
  private double kTurretPosDeadband=0.25;
  private boolean m_finished=false;
  private boolean m_stopWhenNoFuel=true;
  private TargetType m_targetType=TargetType.HUB; // HUB, or PASS = nearest pass target each loop
  private final double kTurretAimToleranceMeters=0.2; // sideways miss at the target the turret gate allows
  private final double kHubEntryHeight=1.8288; // hub opening, 72 in (tools/shotsim/configs/hub.json)
  private final double kLaunchLineSeconds=0.1; // logged launch vector line = launch velocity x this
  public ShotOnTheMoveCommand(CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                          TurretSubsystemPots turret,Limelight limelight, double timeLimit, boolean emptyHopper) {
    this(swerve, shooter, hopper, intake, turret, limelight, timeLimit, emptyHopper, TargetType.HUB);
  }

  /** @param target HUB, or PASS for the nearest pass target each loop */
  public ShotOnTheMoveCommand(CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                          TurretSubsystemPots turret,Limelight limelight, double timeLimit, boolean emptyHopper, TargetType target) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_limelight=limelight;
    m_timeLimit=timeLimit;
    m_stopWhenNoFuel=true;
    overrideDistanceFlag=false;
    
    m_emptyHopper=emptyHopper;
    m_targetType=target;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    // Do not require intake so other commands can control it concurrently
  }

  public ShotOnTheMoveCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                            double timeLimit) {
    this(turret, swerve, shooter, hopper, intake, timeLimit, TargetType.HUB);
  }

  /** @param target HUB, or PASS for the nearest pass target each loop */
  public ShotOnTheMoveCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                            double timeLimit, TargetType target) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_timeLimit=timeLimit;
    m_stopWhenNoFuel=false;
    m_targetType=target;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    // Do not require intake so other commands can control it concurrently
    //DO NOT REQUIRE TURRET OR DRIVE
  }

  private boolean checkNoFuelorFuelTimeLimit(){
    boolean returnValue=false;
    double currentTime;
        if(m_stopWhenNoFuel && m_hopper.getNoFuel()) {
          //System.out.println("No Fuel Detected...");
          returnValue=true;
        }
        //check time limit if the value is greater than zero
        //acts a failsafe if FuelSensor is not working
        currentTime = Timer.getFPGATimestamp();
        if(m_timeLimit>0 && (currentTime>=startShootTime+m_timeLimit)){
          //System.out.println("Shoot Time Limit Reached... Ending Shoot Command");
          returnValue=true;
        }
        return returnValue;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=shooterStates.SPINUP;
    m_finished=false;
    startShootTime=Timer.getFPGATimestamp();
    stateStartTime=startShootTime;
    SmartDashboard.putString("SOTM/State",m_state.toString());
    Logger.recordOutput("Shooter/SOTM/State",m_state.toString());
    if (m_swerve != null) {
      m_swerve.setGasPedalMult(Constants.DriveConstants.kSOTMDriveMultiplier, Constants.DriveConstants.kSOTMTurnMultiplier);
    }

    // Shot map A/B switch and 4414's two live trims; defaults are published once so they can be edited
    ShooterCalibration cal = ShotMaps.get().calibration();
    if (!SmartDashboard.containsKey("SOTM/UseShotMap")) {
      SmartDashboard.putBoolean("SOTM/UseShotMap", ShotMaps.get().available());
    }
    if (cal != null) {
      if (!SmartDashboard.containsKey("SOTM/MapMode")) {
        SmartDashboard.putString("SOTM/MapMode", "AUTO");
      }
      if (!SmartDashboard.containsKey("SOTM/LatencySec")) {
        SmartDashboard.putNumber("SOTM/LatencySec", cal.phaseDelay() + cal.mechLatency());
      }
      if (!SmartDashboard.containsKey("SOTM/HoodOffsetDeg")) {
        SmartDashboard.putNumber("SOTM/HoodOffsetDeg", 0);
      }
      if (!SmartDashboard.containsKey("SOTM/ExitSpeedScale")) {
        SmartDashboard.putNumber("SOTM/ExitSpeedScale", cal.exitSpeedScale());
      }
      cal.setHoodOffsetDeg(SmartDashboard.getNumber("SOTM/HoodOffsetDeg", 0));
      cal.setExitSpeedScale(SmartDashboard.getNumber("SOTM/ExitSpeedScale", cal.exitSpeedScale()));
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double currentTime;
  boolean returnValue=false;

var pose = m_swerve.getPose2d();
ChassisSpeeds robotRelativeSpeeds = m_swerve.getChassisSpeeds();
ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
TargetType target = (m_targetType == TargetType.PASS) ? TurretUtil.getNearestPassTargetType(pose) : m_targetType;

Translation2d turretFieldVelocity = TurretUtil.getFieldRelativeTurretVelocity(pose, robotRelativeSpeeds);
double turretVelX = turretFieldVelocity.getX();
double turretVelY = turretFieldVelocity.getY();
// A/B: shot map (4414 method) or the old lookup-table lead solution
boolean useShotMap = SmartDashboard.getBoolean("SOTM/UseShotMap", ShotMaps.get().available());
ShotSolution targetShot = useShotMap
    ? TurretUtil.computeShotMapSolution(pose, robotRelativeSpeeds, target)
    : TurretUtil.computeLeadShotSolution(pose,turretVelX,turretVelY,target); 
ShotSolution staticShot = TurretUtil.computeShotSolution(pose, target);

double targetspeed=targetShot.shooterSpeedRPS;
double hoodPos=targetShot.trajectoryAngleDegrees;
double turretSourceAngle=targetShot.turretAngleDegrees;

//REFERENCE ONLY - DO NOT USE TARGETING
 double StaticTurretAngleTarget=TurretUtil.get5454TurretAngle(pose,target);
  //always adjust the angle
   double angle=TurretUtil.get5454TurretAngleFromAngle(turretSourceAngle);
         
  //System.out.println("Shooting On the Move - Speed "  + targetspeed  + 
  //                   "Target Angle:" + angle + " Static Angle:" + StaticTurretAngleTarget +" - State:" + m_state);
  SmartDashboard.putNumber("Turret Util Target Angle",angle);
  double targetPos=m_turret.getTargetMotorPosition(angle);
  SmartDashboard.putNumber("Turret Util Target Pos",targetPos); 
  boolean ready = isShotReady(targetShot, targetPos);
  publishDebug(targetShot, staticShot, robotRelativeSpeeds, fieldRelativeSpeeds, turretVelX, turretVelY, angle, targetPos, target);
  m_turret.moveMotor(targetPos); 
  
switch(m_state){
    case SPINUP:
         m_shooter.HoodSetPos(hoodPos);
          
         m_shooter.runNewShooter(targetspeed,
                            0);
          m_state=shooterStates.WAIT;
   
    break;
    case WAIT:
              m_shooter.HoodSetPos(hoodPos);
              m_shooter.runNewShooter(targetspeed, 0);
   
   
        if(ready){
            startShootTime=Timer.getFPGATimestamp();
            m_state=shooterStates.SHOOT;
            stateStartTime=Timer.getFPGATimestamp();
        } 
        
        break;
    case SHOOT:
          // Readiness is re-checked every loop: hold the fuel back while not ready, but stay in SHOOT
          feedFuel(ready, targetspeed, true);
          m_shooter.HoodSetPos(hoodPos);
         //STAY IN SHOOT
          if(checkNoFuelorFuelTimeLimit()){
            m_state=shooterStates.NOFUEL;
          }
       break;
    case NOFUEL:
        fuelcheckStartTime=Timer.getFPGATimestamp();
        if(m_emptyHopper){
          m_flipCount=0; 
          m_flipCountLimit=0;
          m_flipSpeed=0;
          m_HopperPulls=0;
          m_state=shooterStates.EMPTYHOPPER;    
          //m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode);      
         } else{
          m_state=shooterStates.END;          
         }
       break; 
    case EMPTYHOPPER:
         m_shooter.HoodSetPos(hoodPos);
         feedFuel(ready, targetspeed, false); // this state runs the intake itself
    
    /*  //System.out.println("Flip Count"+ m_flipCount);
        m_flipCount=m_flipCount+1;
        if (m_flipCount==m_flipCountLimit){
          //make it twice as fast
          m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode * 3 *  m_flipSpeed);
          m_flipCount = 0;
          m_flipSpeed=m_flipSpeed*-1; // FLIP SIGN TO REVERSE
          if(m_flipCountLimit<kflipCountMax){
            m_flipCountLimit=m_flipCountLimit+10;
          }
        }
        /*if(m_intake.isAtInLimit() || m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)){
          m_state=shooterStates.NOFUEL2NDCHECK;
          m_intake.stopFold();
        }*/
        //if flip count (times through the loop) is greeater than limit than move to next hopper
        if(m_flipCount>m_flipCountLimit){
          m_flipCount=0;
          m_HopperPulls=m_HopperPulls+1;
        }
        
        switch(m_HopperPulls){
          case 0:
            m_flipSpeed=-0.8; // coming in speed
            m_flipCountLimit=3;
          break;
          case 1:
            m_flipSpeed=0.8;  //Out speed
            m_flipCountLimit=2;
          break;
          case 2:
            m_flipSpeed=-0.8;
            m_flipCountLimit=8;
          break;
          case 3:
            m_flipSpeed=0.8; 
            m_flipCountLimit=6;
          break;
          case 4:
            m_flipSpeed=-0.8;
            m_flipCountLimit=12;
          break;
          default:
            if (m_HopperPulls % 2 != 0) {  //EDIT if we change hopper pull limit
              m_flipSpeed=0.8; //Out speed which we should start with first in the default case since we end with an inward pull
              m_flipCountLimit=4;
            } else {
              m_flipSpeed=-0.8; // coming in speed
              m_flipCountLimit=6; // We pull in further then we push out incase we had jamed and never pulled in enough to start
            }
          break;
        }


        m_flipCount=m_flipCount+1;
        
        //System.out.println("Flip Count:" + m_flipCount + " Hopper Pulls: "+ m_HopperPulls + " Speed:"+ m_flipSpeed);
   
        if (m_intake.getCurrentCommand() == null) {
          m_intake.inFold(m_flipSpeed);
        
          if(m_intake.isinNoFlyZone()){
            m_intake.stopIntake();
          } else {
            m_intake.runIntake(Constants.IntakeConstants.highSpeed);
          }
        }
        
        if(m_HopperPulls>kHopperPullLimit){
          //System.out.println("Stop Folding");
          if (m_intake.getCurrentCommand() == null) {
            m_intake.stopFold();
          }
          m_state=shooterStates.SHOOTMORE;
        }
      break;
    case SHOOTMORE:
                  m_shooter.HoodSetPos(hoodPos);
                  feedFuel(ready, targetspeed, false);
   
        //STAY IN THE LOOP FOREVER UNTIL USER STOPS
     break;
    case NOFUEL2NDCHECK:
        currentTime=Timer.getFPGATimestamp();
        if(currentTime>fuelcheckStartTime+kfuelcheckWait){
          if(checkNoFuelorFuelTimeLimit() ){
            m_state=shooterStates.END;
          }
        }
       break;
    case END:
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    m_finished=returnValue;
    SmartDashboard.putString("SOTM/State",m_state.toString());
    Logger.recordOutput("Shooter/SOTM/State",m_state.toString());

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //System.out.println("Stopping Shooter");
    m_shooter.hoodMoveToZero();
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    if (m_intake.getCurrentCommand() == null) {
      m_intake.stopFold();
      m_intake.SetIntakeOutMode();
      m_intake.stopIntake();
    }
    if (m_swerve != null) {
      m_swerve.setGasPedalMult(1.0, 1.0);
    }
    SmartDashboard.putBoolean("SOTM/Interrupted",interrupted);
    Logger.recordOutput("Shooter/SOTM/Interrupted",interrupted);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }

  private boolean checkTurretPos(double targetPosition){
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return true;
    }
    boolean returnValue=false;
    double actual=m_turret.getCurrentPosition();
    if(Math.abs(targetPosition-actual)<kTurretPosDeadband){
      returnValue=true;
    }
    return returnValue;
  }

  /**
   * Runs the kicker, hopper and (when nothing else owns it) the intake while the shot is ready and
   * stops them while it is not. The flywheel keeps tracking the target speed either way.
   */
  private void feedFuel(boolean ready, double targetspeed, boolean runIntake){
    boolean wrapping = m_turret != null && m_turret.isWrappingAround();
    double currentKickerSpeed = (ready && !wrapping) ? Constants.ShooterConstants.KickerSpeed : 0;
    m_shooter.runKicker(currentKickerSpeed);
    m_shooter.runNewShooter(targetspeed, currentKickerSpeed);
    if (ready) {
      m_hopper.agitate(wrapping ? Constants.HopperConstants.agitateLowSpeed : Constants.HopperConstants.agitateSpeed);
    } else {
      m_hopper.stopAgitate();
    }
    if (runIntake && m_intake.getCurrentCommand() == null) {
      if (ready) {
        m_intake.runIntake(Constants.IntakeConstants.highSpeed);
      } else {
        m_intake.stopIntake();
      }
    }
  }

  /**
   * True when a ball fed now should score; logs every gate. Shot-map shots allow half of the map's
   * scoring margins on the flywheel and hood, and atan(0.2 m / d) on the turret. Lookup-table shots
   * keep the old gates (atTargetSpeed + turret deadband). Either way the turret target must be
   * inside the turret's soft limits, since moveMotor ignores anything else.
   */
  private boolean isShotReady(ShotSolution shot, double targetMotorPosition){
    boolean reachable = m_turret.isMotorTargetReachable(targetMotorPosition);
    boolean valid;
    boolean flywheelReady;
    boolean hoodReady;
    boolean turretReady;
    SotmSolution sotm = shot.sotm;
    ShooterCalibration cal = ShotMaps.get().calibration();
    if (sotm != null && cal != null) {
      ShotMapOutput map = sotm.map();
      double flywheelTolRps = Math.min(Constants.ShooterConstants.shooterVelocityLowDeadband,
          cal.rpsForExitSpeed(0.5 * Math.min(map.speedMarginLow(), map.speedMarginHigh())));
      double hoodTolRot = 0.5 * Math.min(map.angleMarginLow(), map.angleMarginHigh()) / Math.abs(cal.degPerRot());
      double turretTolRot = Math.toDegrees(Math.atan(kTurretAimToleranceMeters / sotm.distance()))
          / TurretSubsystemPots.kMotorRotationsToAngle;
      // Same as atTargetSpeed / checkTurretPos: the sim does not model these mechanisms closely
      boolean sim = edu.wpi.first.wpilibj.RobotBase.isSimulation();
      valid = shot.isValid;
      flywheelReady = sim || Math.abs(m_shooter.getShooterSpeed() - sotm.flywheelRps()) <= flywheelTolRps;
      hoodReady = sim || Math.abs(m_shooter.getHoodPos() - sotm.hoodRot()) <= hoodTolRot;
      turretReady = sim || Math.abs(m_turret.getCurrentPosition() - targetMotorPosition) <= turretTolRot;
      Logger.recordOutput("Shooter/SOTM/Ready/FlywheelTolRps", flywheelTolRps);
      Logger.recordOutput("Shooter/SOTM/Ready/HoodTolRot", hoodTolRot);
      Logger.recordOutput("Shooter/SOTM/Ready/TurretTolRot", turretTolRot);
    } else {
      valid = true; // the lookup-table path never gated on isValid
      flywheelReady = m_shooter.atTargetSpeed(shot.shooterSpeedRPS);
      hoodReady = true;
      turretReady = checkTurretPos(targetMotorPosition);
    }
    boolean ready = valid && reachable && flywheelReady && hoodReady && turretReady;

    Logger.recordOutput("Shooter/SOTM/Ready/Valid", valid);
    Logger.recordOutput("Shooter/SOTM/Ready/TurretReachable", reachable);
    Logger.recordOutput("Shooter/SOTM/Ready/Flywheel", flywheelReady);
    Logger.recordOutput("Shooter/SOTM/Ready/Hood", hoodReady);
    Logger.recordOutput("Shooter/SOTM/Ready/Turret", turretReady);
    Logger.recordOutput("Shooter/SOTM/Ready/All", ready);
    SmartDashboard.putBoolean("SOTM/Ready", ready);
    return ready;
  }

  private void publishDebug(ShotSolution leadShot, ShotSolution staticShot, ChassisSpeeds robotRelativeSpeeds,
                            ChassisSpeeds fieldRelativeSpeeds, double turretVelX, double turretVelY, double turretAngle5454,
                            double targetMotorPosition, TargetType target){
    double leadAngleDelta = leadShot.turretAngleDegrees - staticShot.turretAngleDegrees;

    SmartDashboard.putNumber("SOTM/DistanceMeters",leadShot.distanceMeters);
    SmartDashboard.putNumber("SOTM/StaticDistanceMeters",staticShot.distanceMeters);
    SmartDashboard.putNumber("SOTM/TimeOfFlightSec",leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/ShooterSpeedRPS",leadShot.shooterSpeedRPS);
    SmartDashboard.putNumber("SOTM/HoodPosition",leadShot.trajectoryAngleDegrees);
    SmartDashboard.putNumber("SOTM/TurretAngleRawDeg",leadShot.turretAngleDegrees);
    SmartDashboard.putNumber("SOTM/TurretAngle5454Deg",turretAngle5454);
    SmartDashboard.putNumber("SOTM/TurretMotorTarget",targetMotorPosition);
    SmartDashboard.putNumber("SOTM/TurretMotorActual",m_turret.getCurrentPosition());
    SmartDashboard.putBoolean("SOTM/TurretAtTarget",checkTurretPos(targetMotorPosition));
    SmartDashboard.putBoolean("SOTM/ShotValid",leadShot.isValid);
    SmartDashboard.putNumber("SOTM/LeadAngleDeltaDeg",leadAngleDelta);
    SmartDashboard.putNumber("SOTM/LeadDistanceMeters",leadShot.leadDistanceMeters);
    SmartDashboard.putNumber("SOTM/AimBehindXMeters",-turretVelX * leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/AimBehindYMeters",-turretVelY * leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/PredictedTurretX",leadShot.predictedTurretX);
    SmartDashboard.putNumber("SOTM/PredictedTurretY",leadShot.predictedTurretY);
    SmartDashboard.putNumber("SOTM/RobotRelVxMps",robotRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotRelVyMps",robotRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotOmegaRadPerSec",robotRelativeSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("SOTM/FieldRelVxMps",fieldRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM/FieldRelVyMps",fieldRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SOTM/TurretFieldVelXMps",turretVelX);
    SmartDashboard.putNumber("SOTM/TurretFieldVelYMps",turretVelY);

    Logger.recordOutput("Shooter/SOTM/DistanceMeters",leadShot.distanceMeters);
    Logger.recordOutput("Shooter/SOTM/StaticDistanceMeters",staticShot.distanceMeters);
    Logger.recordOutput("Shooter/SOTM/TimeOfFlightSec",leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/ShooterSpeedRPS",leadShot.shooterSpeedRPS);
    Logger.recordOutput("Shooter/SOTM/HoodPosition",leadShot.trajectoryAngleDegrees);
    Logger.recordOutput("Shooter/SOTM/TurretAngleRawDeg",leadShot.turretAngleDegrees);
    Logger.recordOutput("Shooter/SOTM/TurretAngle5454Deg",turretAngle5454);
    Logger.recordOutput("Shooter/SOTM/TurretMotorTarget",targetMotorPosition);
    Logger.recordOutput("Shooter/SOTM/TurretMotorActual",m_turret.getCurrentPosition());
    Logger.recordOutput("Shooter/SOTM/TurretAtTarget",checkTurretPos(targetMotorPosition));
    Logger.recordOutput("Shooter/SOTM/ShotValid",leadShot.isValid);
    Logger.recordOutput("Shooter/SOTM/LeadAngleDeltaDeg",leadAngleDelta);
    Logger.recordOutput("Shooter/SOTM/LeadDistanceMeters",leadShot.leadDistanceMeters);
    Logger.recordOutput("Shooter/SOTM/AimBehindXMeters",-turretVelX * leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/AimBehindYMeters",-turretVelY * leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/PredictedTurretX",leadShot.predictedTurretX);
    Logger.recordOutput("Shooter/SOTM/PredictedTurretY",leadShot.predictedTurretY);
    Logger.recordOutput("Shooter/SOTM/RobotRelVxMps",robotRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/RobotRelVyMps",robotRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/RobotOmegaRadPerSec",robotRelativeSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("Shooter/SOTM/FieldRelVxMps",fieldRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/FieldRelVyMps",fieldRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/TurretFieldVelXMps",turretVelX);
    Logger.recordOutput("Shooter/SOTM/TurretFieldVelYMps",turretVelY);
    Logger.recordOutput("Shooter/SOTM/Target",target.toString());
    Logger.recordOutput("Shooter/SOTM/UsingShotMap",leadShot.sotm != null);

    // Shot-map solve (4414 method): map lookup at (d, vr), then vt cancelled with 3D vectors
    SotmSolution sotm = leadShot.sotm;
    if (sotm != null) {
      ShotMapOutput map = sotm.map();
      String reasons = String.join(",", sotm.reasons());
      SmartDashboard.putString("SOTM/MapName",leadShot.shotMapName);
      SmartDashboard.putString("SOTM/Reasons",reasons);
      Logger.recordOutput("Shooter/SOTM/Map/Name",leadShot.shotMapName);
      Logger.recordOutput("Shooter/SOTM/Map/Mode",leadShot.shotMapMode.toString());
      Logger.recordOutput("Shooter/SOTM/Map/Distance",sotm.distance());
      Logger.recordOutput("Shooter/SOTM/Map/RadialVelocity",sotm.radialVelocity());
      Logger.recordOutput("Shooter/SOTM/Map/TangentialVelocity",sotm.tangentialVelocity());
      Logger.recordOutput("Shooter/SOTM/Map/ElevationDeg",sotm.elevationDeg());
      Logger.recordOutput("Shooter/SOTM/Map/ExitSpeed",sotm.exitSpeed());
      Logger.recordOutput("Shooter/SOTM/Map/HoodRot",sotm.hoodRot());
      Logger.recordOutput("Shooter/SOTM/Map/FlywheelRps",sotm.flywheelRps());
      Logger.recordOutput("Shooter/SOTM/Map/Tof",sotm.tof());
      Logger.recordOutput("Shooter/SOTM/Map/AngleMarginLow",map.angleMarginLow());
      Logger.recordOutput("Shooter/SOTM/Map/AngleMarginHigh",map.angleMarginHigh());
      Logger.recordOutput("Shooter/SOTM/Map/SpeedMarginLow",map.speedMarginLow());
      Logger.recordOutput("Shooter/SOTM/Map/SpeedMarginHigh",map.speedMarginHigh());
      Logger.recordOutput("Shooter/SOTM/Map/VtMax",map.vtMax());
      Logger.recordOutput("Shooter/SOTM/Map/Valid",sotm.valid());
      Logger.recordOutput("Shooter/SOTM/Map/Reasons",reasons);
    }

    ShooterCalibration cal = ShotMaps.get().calibration();
    if (cal == null) {
      return; // the 3D lines need the release height and the hood calibration
    }
    double releaseZ = cal.axleHeight();
    Pose2d turretPose = TurretUtil.getTurretPose(m_swerve.getPose2d());
    Pose2d targetPose2d = TurretUtil.getTargetPose(target);
    double targetZ = (target == TargetType.HUB) ? kHubEntryHeight : 0;

    if (sotm != null) {
      // Launch velocity relative to the robot, drawn from the release point
      Translation3d release = new Translation3d(sotm.turretPosition().getX(), sotm.turretPosition().getY(), releaseZ);
      Logger.recordOutput("Shooter/SOTM/Map/LaunchLine3d",
          new Translation3d[] { release, release.plus(sotm.launchVector().times(kLaunchLineSeconds)) });
    }

    // Ideal Aim Line (Turret to target)
    double targetTurretHeadingRad = m_swerve.getPose2d().getRotation().getRadians() + Math.toRadians(leadShot.turretAngleDegrees);
    Pose3d idealStart3d = new Pose3d(turretPose.getX(), turretPose.getY(), releaseZ, new Rotation3d(0, 0, targetTurretHeadingRad));
    Pose3d idealEnd3d = new Pose3d(targetPose2d.getX(), targetPose2d.getY(), targetZ, new Rotation3d());
    Logger.recordOutput("Shooter/SOTM/AimLine3d", new Pose3d[] { idealStart3d, idealEnd3d });

    // Actual Shot Vector based on REAL physical turret motor position, hood angle, and flywheel power
    double actualMotorRotations = m_turret.getCurrentPosition();
    double actual5454Deg = actualMotorRotations >= 0 
        ? actualMotorRotations * TurretSubsystemPots.kMotorRotationsToAngle 
        : 360.0 + (actualMotorRotations * TurretSubsystemPots.kMotorRotationsToAngle);
    double actualTurretRelDeg = (actual5454Deg > 180.0) ? (360.0 - actual5454Deg) : -actual5454Deg;

    double robotHeadingRad = m_swerve.getPose2d().getRotation().getRadians();
    double actualTurretFieldHeadingRad = robotHeadingRad + Math.toRadians(actualTurretRelDeg);

    double currentRps = Math.abs(m_shooter.getShooterSpeed());
    double targetRps = leadShot.shooterSpeedRPS;
    double powerRatio = (targetRps > 5.0) ? Math.min(Math.max(currentRps / targetRps, 0.0), 1.2) : 1.0;
    
    double distance = (leadShot.distanceMeters > 0 && leadShot.distanceMeters < 50.0) ? leadShot.distanceMeters : 3.0;
    double actualShotDistanceMeters = Math.max(0.5, distance * powerRatio);

    // getHoodPos() is CANcoder rotations; the calibration turns it into the launch angle
    double hoodElevationRad = Math.toRadians(cal.angleForHoodRot(m_shooter.getHoodPos()));

    double startX = turretPose.getX();
    double startY = turretPose.getY();
    double startZ = releaseZ;

    double endX = startX + actualShotDistanceMeters * Math.cos(hoodElevationRad) * Math.cos(actualTurretFieldHeadingRad);
    double endY = startY + actualShotDistanceMeters * Math.cos(hoodElevationRad) * Math.sin(actualTurretFieldHeadingRad);
    double endZ = startZ + actualShotDistanceMeters * Math.sin(hoodElevationRad);

    Pose3d actualShotStart3d = new Pose3d(startX, startY, startZ, new Rotation3d(0, -hoodElevationRad, actualTurretFieldHeadingRad));
    Pose3d actualShotEnd3d = new Pose3d(endX, endY, endZ, new Rotation3d());

    Logger.recordOutput("Shooter/SOTM/ActualShotLine3d", new Pose3d[] { actualShotStart3d, actualShotEnd3d });
    Logger.recordOutput("Shooter/SOTM/ActualTurretFieldHeadingDeg", Math.toDegrees(actualTurretFieldHeadingRad));
    Logger.recordOutput("Shooter/SOTM/FlywheelPowerRatio", powerRatio);
  }
  
}

