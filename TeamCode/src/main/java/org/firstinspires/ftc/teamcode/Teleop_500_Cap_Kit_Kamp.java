package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Teleop_Enhanced provides enhanced teleoperated control for the robot during driver-controlled periods.
 * 
 * KEY FEATURE: Shooter velocity is capped at 500 RPM maximum - enforced at multiple levels
 * 
 * Features:
 * - Field-centric driving with mecanum wheels
 * - Vision-based auto-alignment using Limelight 3A
 * - Distance-based automatic shooter velocity calculation
 * - Zone-based shooting (close/far)
 * - Intake and indexer control
 * - LED pattern feedback system
 * - Odometry tracking with GoBilda Pinpoint
 * 
 * @author FTC Team 23070 Royal Turtles
 * @version 2.0 Enhanced with 500 RPM velocity cap
 */
@TeleOp
public class Teleop_500_Cap_Kit_Kamp extends LinearOpMode {

    // =====================================================================
    // CONSTANTS - VELOCITY CAP ENFORCED HERE
    // =====================================================================
    public static class Constants {
        // *** SHOOTER VELOCITY CAP AT 500 RPM ***
        // These velocities are all capped at maximum 500 RPM
        public static final double MAX_SHOOTER_VELOCITY = 500.0;  // HARD CAP
        
        // Shooter - all velocities constrained to max 500
        public static final double CLOSE_ZONE_VELOCITY = 400.0;  // Previously 1230, now 400
        public static final double FAR_ZONE_VELOCITY = 450.0;    // Previously 1500, now 450
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;
        public static final int SHOOTER_TOLERANCE = 20;          // Reduced tolerance for lower speeds
        public static int SHOOTER_PLUS = 25;                     // Reduced from 50

        // Drive
        public static final double PIVOT_MULTIPLIER = 0.8;
        public static final double STRAFE_CORRECTION = 1.1;
        public static final double AUTO_ALIGN_TOLERANCE = 0.2;
        public static final double ALIGN_P_GAIN = 0.03;

        // Intake
        public static final double INTAKE_POWER = 1.0;
        public static final double INTAKE_REVERSE_POWER = -0.7;
        public static final double TRIGGER_THRESHOLD = 0.2;

        // Index
        public static final int INDEX_STEP = 280;
        public static final double PASSIVE_INDEX_VELOCITY = 20;
        public static double MAG_DUMP_POWER = 0.8;              // Reduced from 1.0

        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;
        public static final double TX_OFFSET_DEGREES_CLOSE = 4;
        public static final double TX_OFFSET_DEGREES_FAR = 0;

        // Pinpoint
        public static final double PINPOINT_X_OFFSET = -107.31371;
        public static final double PINPOINT_Y_OFFSET = 0.0;

        // AutoShoot lookup table - all velocities capped at 500
        public static final double[] DISTANCES = {27, 29.25, 30.2, 31, 32, 34, 35, 46, 47, 48, 9999};
        public static final double[] VELOCITIES = {
            Math.min(1055 - 17, MAX_SHOOTER_VELOCITY),
            Math.min(1150 - 15, MAX_SHOOTER_VELOCITY),
            Math.min(1150 - 17, MAX_SHOOTER_VELOCITY),
            Math.min(1125 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1150 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1200 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1200 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1400 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1400 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1400 - 20, MAX_SHOOTER_VELOCITY),
            Math.min(1045 - 20, MAX_SHOOTER_VELOCITY)
        };

        // Distance sensor
        public static final double ACTIVATION_DISTANCE = 3.3; // INCHES
    }

    // =====================================================================
    // INSTANCE VARIABLES
    // =====================================================================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime autoIndexAlertTimeout = new ElapsedTime();
    double direction_x, direction_y, pivot, heading;
    double FLPower, FRPower, BLPower, BRPower, shooterTargetVel;
    boolean doAutoIndex = false;
    boolean lastDistanceGood = false;

    // Hardware
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // Gamepad state
    Gamepad gamepad;

    // Zone selection
    boolean closeZone = true;
    boolean farZone = false;

    // PID state
    double shooterIntegral = 0;
    double shooterLastError = 0;

    // Shooter velocity model
    ShooterVelocityLookupTable shooterVelocityTable;

    // =====================================================================
    // MAIN OPMODE
    // =====================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initializeHardware();

        // Initialize lookup table
        shooterVelocityTable = new ShooterVelocityLookupTable(Constants.DISTANCES, Constants.VELOCITIES);

        gamepad = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("⚠️  SHOOTER VELOCITY CAP", Constants.MAX_SHOOTER_VELOCITY + " RPM");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();
        pinpoint.resetPosAndIMU();
        setShooterVel(Constants.CLOSE_ZONE_VELOCITY);
        closeZone = true;

        // =========================================================
        // MAIN LOOP
        // =========================================================
        while (opModeIsActive()) {
            pinpoint.update();
            gamepad.copy(gamepad1);

            // --- DRIVING ---
            direction_x = gamepad.left_stick_x;
            direction_y = -gamepad.left_stick_y;
            pivot = gamepad.right_stick_x * Constants.PIVOT_MULTIPLIER;
            heading = pinpoint.getHeading(AngleUnit.RADIANS);

            if (gamepad.cross) {
                telemetry.addData("Auto Align Status: ", autoAlign(Constants.AUTO_ALIGN_TOLERANCE));
                double predictedVel = shooterVelocityTable.predict(getDistanceToTag());
                setShooterVel(capShooterVelocity(predictedVel + Constants.SHOOTER_PLUS));
            } else {
                fieldCentric(direction_y, direction_x, pivot, heading);
            }

            // --- IMU RESET ---
            if (gamepad.optionsWasPressed()) {
                pinpoint.resetPosAndIMU();
                sleep(250);
            }

            // --- ZONE SELECTION ---
            if (gamepad.dpadUpWasPressed()) {
                closeZone = true;
                farZone = false;
                Constants.MAG_DUMP_POWER = 0.8;
                setShooterVel(Constants.CLOSE_ZONE_VELOCITY);
            } else if (gamepad.dpadDownWasPressed()) {
                farZone = true;
                closeZone = false;
                Constants.MAG_DUMP_POWER = 0.7;
                setShooterVel(Constants.FAR_ZONE_VELOCITY);
            }

            // Adaptive power adjustment based on distance
            if (getDistanceToTag() > 40 && getDistanceToTag() != Double.POSITIVE_INFINITY) {
                Constants.MAG_DUMP_POWER = 0.6;
                Constants.SHOOTER_PLUS = 20;
            } else {
                Constants.MAG_DUMP_POWER = 0.8;
                Constants.SHOOTER_PLUS = 25;
            }

            // --- SHOOTER ---
            if (gamepad.triangle) {
                double predictedVel = shooterVelocityTable.predict(getDistanceToTag());
                setShooterVel(capShooterVelocity(predictedVel + Constants.SHOOTER_PLUS));
            }

            if (gamepad.rightBumperWasPressed()) {
                if (closeZone) {
                    setShooterVel(Constants.CLOSE_ZONE_VELOCITY);
                } else if (farZone) {
                    setShooterVel(Constants.FAR_ZONE_VELOCITY);
                }
            } else if (gamepad.leftBumperWasPressed()) {
                lShooter.setPower(0);
                rShooter.setPower(0);
            }

            // --- INTAKE & INDEX ---
            if (gamepad.right_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_POWER);
                index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                index.setVelocity(Constants.PASSIVE_INDEX_VELOCITY);
            } else if (gamepad.left_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_REVERSE_POWER);
            } else if (gamepad.dpadRightWasPressed()) {
                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP);
                intake.setPower(1.0);
            } else if (gamepad.dpadLeftWasPressed()) {
                setIndexPos(index.getCurrentPosition() - Constants.INDEX_STEP);
                intake.setPower(-1.0);
            } else if (gamepad.circle) {
                index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                index.setPower(Constants.MAG_DUMP_POWER);
                intake.setPower(Constants.MAG_DUMP_POWER);
            } else {
                index.setPower(1.0);
                index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                index.setTargetPosition(index.getCurrentPosition());
                intake.setPower(0);
            }

            // --- BLINKIN LED FEEDBACK ---
            updateLEDPattern();

            // --- TELEMETRY ---
            telemetry.addData("=== SHOOTER VELOCITY CAP ===", Constants.MAX_SHOOTER_VELOCITY + " RPM");
            telemetry.addData("Heading (DEGREES)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter Velocity", getAvgShooterVel());
            telemetry.addData("Target Velocity (Capped)", capShooterVelocity(shooterTargetVel));
            telemetry.addData("Distance to Tag", String.format("%.2f", getDistanceToTag()));
            telemetry.addData("Zone", closeZone ? "CLOSE" : farZone ? "FAR" : "NONE");
            telemetry.addData("Within Tolerance", shooterWithinTolerance(shooterTargetVel) ? "✓ YES" : "✗ NO");
            telemetry.update();
        }
    }

    // =====================================================================
    // DRIVE METHODS
    // =====================================================================
    /**
     * Drives the robot using field-centric control with mecanum wheels
     */
    public void fieldCentric(double y, double x, double rx, double heading) {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Rotate input by current heading for field-centric control
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * Constants.STRAFE_CORRECTION;

        // Normalize wheel powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        FLPower = (rotY + rotX + rx) / denominator;
        FRPower = (rotY - rotX - rx) / denominator;
        BLPower = (rotY - rotX + rx) / denominator;
        BRPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(FLPower);
        frontRight.setPower(FRPower);
        backLeft.setPower(BLPower);
        backRight.setPower(BRPower);
    }

    /**
     * Automatically aligns the robot to face the target using Limelight
     */
    public String autoAlign(double tolerance) {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            stopDrivetrain();
            return "RESULT NULL";
        } else if (!result.isValid()) {
            stopDrivetrain();
            return "RESULT INVALID";
        }

        double tx = result.getTx() - (getDistanceToTag() > 40 ? Constants.TX_OFFSET_DEGREES_CLOSE : Constants.TX_OFFSET_DEGREES_FAR);
        if (Math.abs(tx) <= tolerance) {
            stopDrivetrain();
            return "SUCCESS";
        }

        double pivot = Math.max(-1.0, Math.min(1.0, Constants.ALIGN_P_GAIN * tx));

        fieldCentric(0, 0, pivot, 0);

        return "IN PROGRESS";
    }

    private void stopDrivetrain() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // =====================================================================
    // SHOOTER METHODS - VELOCITY CAP ENFORCED
    // =====================================================================
    
    /**
     * CRITICAL: Caps shooter velocity to maximum of 500 RPM
     * This method must be called before setting any shooter velocity
     */
    public double capShooterVelocity(double vel) {
        return Math.min(vel, Constants.MAX_SHOOTER_VELOCITY);
    }

    /**
     * Sets shooter velocity with hard cap at 500 RPM
     */
    public void setShooterVel(double vel) {
        double cappedVel = capShooterVelocity(vel);
        shooterTargetVel = cappedVel;
        lShooter.setVelocity(cappedVel);
        rShooter.setVelocity(cappedVel);
    }

    /**
     * Sets shooter using PID with velocity capped
     */
    public void setShooterVelWithPower(double vel) {
        double cappedVel = capShooterVelocity(vel);
        shooterTargetVel = cappedVel;
        lShooter.setPower(shooterPID(cappedVel, lShooter.getVelocity()));
        rShooter.setPower(shooterPID(cappedVel, rShooter.getVelocity()));
    }

    /**
     * Gets average velocity from both shooter motors
     */
    public int getAvgShooterVel() {
        return (int) ((lShooter.getVelocity() + rShooter.getVelocity()) / 2);
    }

    /**
     * Checks if shooter is within tolerance of target
     */
    public boolean shooterWithinTolerance(double target) {
        double cappedTarget = capShooterVelocity(target);
        return withinTolerance(getAvgShooterVel(), cappedTarget, Constants.SHOOTER_TOLERANCE);
    }

    /**
     * Generic tolerance checking
     */
    public boolean withinTolerance(double val, double target, double tol) {
        return Math.abs(target - val) <= tol;
    }

    /**
     * PID controller for shooter velocity
     */
    public double shooterPID(double targetVel, double currentVel) {
        double error = targetVel - currentVel;
        double proportional = Constants.SHOOTER_P_GAIN * error;

        shooterIntegral += error * 0.02; // Approximate dt
        double integralTerm = Constants.SHOOTER_I_GAIN * shooterIntegral;

        double derivative = (error - shooterLastError) / 0.02;
        double derivativeTerm = Constants.SHOOTER_D_GAIN * derivative;
        shooterLastError = error;

        double output = proportional + integralTerm + derivativeTerm;
        return Math.min(Math.max(output, -1), 1);
    }

    // =====================================================================
    // INDEX METHODS
    // =====================================================================
    public void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    // =====================================================================
    // LIMELIGHT VISION METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (isLatestResultValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(Constants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (Constants.GOAL_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return Double.POSITIVE_INFINITY;
    }

    public boolean isLatestResultValid() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // =====================================================================
    // LED FEEDBACK SYSTEM
    // =====================================================================
    private void updateLEDPattern() {
        if (shooterTargetVel > 100 &&
                (lShooter.getVelocity() < 20 || rShooter.getVelocity() < 20)) {
            // Shooter spinning up but not at speed - violet
            setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if ((gamepad.triangle || gamepad.cross) && !isLatestResultValid()) {
            // Trying to shoot but no vision target - blue
            setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (!shooterWithinTolerance(shooterTargetVel)) {
            // Shooter not at target speed - orange
            setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        } else if (shooterTargetVel > 0 && shooterWithinTolerance(shooterTargetVel)) {
            // Shooter ready - green
            setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            // Idle - rainbow
            setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
    }

    void setPattern(RevBlinkinLedDriver.BlinkinPattern p) {
        pattern = p;
        blinkinLedDriver.setPattern(pattern);
    }

    // =====================================================================
    // SHOOTER VELOCITY LOOKUP TABLE (LINEAR INTERPOLATION)
    // =====================================================================
    public static class ShooterVelocityLookupTable {
        private final double[] distances;
        private final double[] velocities;

        public ShooterVelocityLookupTable(double[] distances, double[] velocities) {
            this.distances = distances;
            this.velocities = velocities;
        }

        /**
         * Predicts shooter velocity from distance using linear interpolation
         * All returned velocities are capped at 500 RPM
         */
        public double predict(double distance) {
            if (Double.isNaN(distance) || Double.isInfinite(distance)) 
                return Constants.CLOSE_ZONE_VELOCITY;
            if (distances == null || velocities == null) 
                return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length == 0 || velocities.length == 0) 
                return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length != velocities.length) 
                return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length == 1) 
                return Math.min(velocities[0], Constants.MAX_SHOOTER_VELOCITY);

            // Clamp out-of-range distances
            if (distance <= distances[0]) 
                return Math.min(velocities[0], Constants.MAX_SHOOTER_VELOCITY);
            
            int last = distances.length - 1;
            if (distance >= distances[last]) 
                return Math.min(velocities[last], Constants.MAX_SHOOTER_VELOCITY);

            // Linear interpolation between points
            for (int i = 0; i < last; i++) {
                double d0 = distances[i];
                double d1 = distances[i + 1];

                if (distance >= d0 && distance <= d1) {
                    double v0 = velocities[i];
                    double v1 = velocities[i + 1];
                    double denom = (d1 - d0);
                    if (Math.abs(denom) < 1e-9) 
                        return Math.min((v0 + v1) * 0.5, Constants.MAX_SHOOTER_VELOCITY);
                    
                    double t = (distance - d0) / denom;
                    double interpolated = v0 + t * (v1 - v0);
                    return Math.min(interpolated, Constants.MAX_SHOOTER_VELOCITY);
                }
            }

            return Math.min(velocities[last], Constants.MAX_SHOOTER_VELOCITY);
        }
    }

    // =====================================================================
    // HARDWARE INITIALIZATION
    // =====================================================================
    void initializeHardware() {
        // --- Drive Motors ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Intake ---
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Indexer/Transfer ---
        index = hardwareMap.get(DcMotorEx.class, "transfer");
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        index.setTargetPosition(0);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // --- Shooter Motors (VELOCITY CAPPED AT 500 RPM) ---
        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Odometry (GoBilda Pinpoint) ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET, Constants.PINPOINT_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // --- Limelight Vision ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        // --- LED Controller ---
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }
}
