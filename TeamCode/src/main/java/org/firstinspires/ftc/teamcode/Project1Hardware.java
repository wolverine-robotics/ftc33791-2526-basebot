package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Project1Hardware manages all hardware components for the robot.
 * 
 * <p>This class provides a centralized interface for initializing and controlling all robot hardware,
 * including the drivetrain, shooter, intake, index, and IMU sensors. It encapsulates hardware
 * initialization, configuration, and provides convenient getter/setter methods for subsystem control.
 * 
 * <p><b>Hardware Components:</b>
 * <ul>
 *   <li>Drivetrain: Four mecanum drive motors (frontLeft, frontRight, backLeft, backRight)</li>
 *   <li>Shooter: Two shooter motors (lShooter, rShooter) for launching game pieces</li>
 *   <li>Intake: Single intake motor for collecting game pieces</li>
 *   <li>Index: Single index motor for positioning game pieces</li>
 *   <li>IMU: Inertial Measurement Unit for orientation and heading tracking</li>
 * </ul>
 * 
 * <p>All motors are configured with BRAKE mode to prevent drifting when power is set to zero.
 * 
 * @author FTC Team 23070 Royal Turtles
 */
public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;

    public double integral = 0;
    public double lastError = 0;

    /**
     * Initializes all hardware components from the hardware map.
     */
    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        index = hardwareMap.get(DcMotorEx.class, "index");
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");


        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        lShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //X offset = sideways (forward pod) | Y offset = forwards (strafe pod)
        pinpoint.setOffsets(-107.31371, 0.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        //Left and Forward are positive
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /**
     * Calculates PID control output for the shooter subsystem.
     * <p><b>Warning:</b> There appears to be a bug in the implementation - the parameter
     * {@code getShooterVel} is treated as a function call but should be a value. This method
     * should likely call {@link #getShooterVel()} instead.
     * 
     * @param shooterTargetVel Target velocity for the shooter in encoder ticks per second
     * @param getShooterVel Current shooter velocity in encoder ticks per second
     *                      (Note: parameter name suggests this should be a method call)
     * @param lastTime Timestamp of the last PID calculation in milliseconds
     * @return PID control output value (should be clamped to [-1, 1] before use)
     */
    public double shooterPIDCalculate(double shooterTargetVel, double getShooterVel, long lastTime) {
        // Calculate error
        double error = shooterTargetVel - getShooterVel;
        // Calculate time difference
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Proportional term
        double proportional = BotConstants.Shooter.SHOOTER_P_GAIN * error;

        // Integral term
        integral += error * dt;
        double integralTerm = BotConstants.Shooter.SHOOTER_I_GAIN * integral;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double derivativeTerm = BotConstants.Shooter.SHOOTER_D_GAIN * derivative;
        lastError = error;

        // Calculate output
        double output = proportional + integralTerm + derivativeTerm;

        // Clamp output to valid range [-1, 1]

        return Math.min(Math.max(output, -1), 1);
    }

    // --- Shooter Setters ---
    
    public void setShooterVel(double vel) {

        lShooter.setPower(shooterPIDCalculate(vel, lShooter.getVelocity(), System.currentTimeMillis()));
        rShooter.setPower(shooterPIDCalculate(vel, rShooter.getVelocity(), System.currentTimeMillis()));
    }

    public void setShooter(double speed) {
        lShooter.setPower(speed);
        rShooter.setPower(speed);
    }
    
    public void shooterOff() {
        lShooter.setPower(0);
        rShooter.setPower(0);
    }

    // --- Shooter Getters ---
    
    public double getShooterVel() {
        return (lShooter.getVelocity() + rShooter.getVelocity()) / 2;
    }
    
    public double getShooterSpeed() {
        return (lShooter.getPower() + rShooter.getPower()) / 2;
    }

    // --- Intake Setters ---
    
    public void setIntakeVel(double vel) {
        intake.setVelocity(vel);
    }
    
    public void setIntake(double speed) {
        intake.setPower(speed);
    }
    
    public void intakeOff() {
        intake.setPower(0);
    }
    
    public void intakeReverse() {
        intake.setPower(-0.7);
    }

    // --- Index Setters ---
    
    public void setIndexVel(double vel) {
        index.setVelocity(vel);
    }
    
    public void setIndex(double speed) {
        index.setPower(speed);
    }
    
    public void indexOff() {
        index.setPower(0);
    }

    /**
     * Moves the index motor to a specific encoder position.
     * 
     * <p>This method switches the motor to RUN_TO_POSITION mode and sets the target position.
     * The motor will run at full power (1.0) until it reaches the target position.
     * 
     * <p><b>Note:</b> The motor mode is changed to RUN_TO_POSITION. If you need to use
     * velocity or power control afterward, you must change the mode back to RUN_WITHOUT_ENCODER.
     * 
     * @param pos Target encoder position in ticks
     */
    public void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    public void indexBallUp() {
        setIndexPos(getIndexPos() + 150);
    }
    
    public void indexBallDown() {
        setIndexPos(getIndexPos() - 150);
    }

    // --- Intake Getters ---
    
    public double getIntakeVel() {
        return intake.getVelocity();
    }
    
    public double getIntakeSpeed() {
        return intake.getPower();
    }

    // --- Index Getters ---
    
    public double getIndexVel() {
        return index.getVelocity();
    }
    
    public double getIndexSpeed() {
        return index.getPower();
    }
    
    public int getIndexPos() {
        return index.getCurrentPosition();
    }

    // --- Limelight Methods ---
    public Pose3D getLimelightBotPose(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                return botpose;
            }
        }
        return null;
    }

    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                double targetOffsetAngle_Vertical = result.getTy();

                double angleToGoalDegrees = BotConstants.Limelight.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                //calculate distance
                double distanceFromLimelightToGoalInches =
                        (BotConstants.Limelight.goalHeightInches - BotConstants.Limelight.limelightLensHeightInches)
                                / Math.tan(angleToGoalRadians);

                return distanceFromLimelightToGoalInches;
            }
        }
        return Double.POSITIVE_INFINITY;
    }
}