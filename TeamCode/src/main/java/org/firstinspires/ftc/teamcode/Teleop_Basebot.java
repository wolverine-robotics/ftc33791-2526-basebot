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
 * Teleop_Basebot provides teleoperated control for the robot during driver-controlled periods.
 * Self-contained file with all necessary classes embedded.
 *
 * @author FTC Team 23070 Royal Turtles
 */
@TeleOp
public class Teleop_Basebot extends LinearOpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    public static class Constants {
        // Shooter
        public static final double CLOSE_ZONE_VELOCITY = 1230;
        public static final double FAR_ZONE_VELOCITY = 1500;
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;
        public static final int SHOOTER_TOLERANCE = 50;
        public static int SHOOTER_PLUS = 50;

        // Drive
        public static final double PIVOT_MULTIPLIER = 0.8;
        public static final double STRAFE_CORRECTION = 1.1;
        public static final double AUTO_ALIGN_TOLERANCE = 0.25;
        public static final double ALIGN_P_GAIN = 0.03;

        // Intake
        public static final double INTAKE_POWER = 1.0;
        public static final double INTAKE_REVERSE_POWER = -0.7;
        public static final double TRIGGER_THRESHOLD = 0.2;

        // Index
        public static final int INDEX_STEP = 280;
        public static final double PASSIVE_INDEX_VELOCITY = 20;
        public static double MAG_DUMP_POWER = 0.9;


        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;
        //If turns left, positive is more left
        //If turns right, positive is more right (edit these comments to confirm)
        public static final double TX_OFFSET_DEGREES_CLOSE = 5;
        public static final double TX_OFFSET_DEGREES_FAR = 0;


        // Pinpoint
        public static final double PINPOINT_X_OFFSET = -107.31371;
        public static final double PINPOINT_Y_OFFSET = 0.0;

        // AutoShoot lookup table data (distance in inches -> shooter velocity)
        public static final double[] DISTANCES = {27, 29.25, 30.2, 31, 32, 34, 35, 46, 47, 48, 9999};
        public static final double[] VELOCITIES = {1055, 1150, 1150, 1125, 1150, 1200, 1200, 1400, 1400, 1400, 1045};

        // Distance sensor
        public static final double ACTIVATION_DISTANCE = 3.3; //INCHES
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
//    DistanceSensor distance;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // Gamepad state
    Gamepad gamepad;
//    Gamepad lastGamepad;

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
//        lastGamepad = new Gamepad();

        telemetry.addData("Status", "Initialized");
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
                setShooterVel(predictedVel + Constants.SHOOTER_PLUS);
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

            if (getDistanceToTag() > 40 && getDistanceToTag() != Double.POSITIVE_INFINITY) {
                Constants.MAG_DUMP_POWER = 0.6;
                Constants.SHOOTER_PLUS = 50;
            } else {
                Constants.MAG_DUMP_POWER = 0.9;
                Constants.SHOOTER_PLUS = 85;
            }

            // --- SHOOTER ---
            if (gamepad.triangle) {
                double predictedVel = shooterVelocityTable.predict(getDistanceToTag());
                setShooterVel(predictedVel + Constants.SHOOTER_PLUS);
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

            // --- INTAKE ---
            // --- INDEX ---
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
            } else if (gamepad.squareWasPressed()) {
//                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP * 3);
//                intake.setPower(1.0);
            } else if (gamepad.circle) {
                index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                index.setPower(Constants.MAG_DUMP_POWER);
                intake.setPower(Constants.MAG_DUMP_POWER);
            } else   {
                index.setPower(1.0);
                index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                index.setTargetPosition(index.getCurrentPosition());
                intake.setPower(0);
            }

            // --- BLINKIN ---
            if (shooterTargetVel > 500 &&
                    (lShooter.getVelocity() < 50 || rShooter.getVelocity() < 50)) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if ((gamepad.triangle || gamepad.cross) && !isLatestResultValid()) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (!shooterWithinTolerance(shooterTargetVel)) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            } else if (shooterWithinTolerance(shooterTargetVel)) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            // --- DISTANCE ---
//            if (withinDistance()) {
//                if (!lastDistanceGood) {
//                    autoIndexAlertTimeout.reset();
//                    lastDistanceGood = true;
//                }
//            } else {
//                lastDistanceGood = false;
//            }

            // --- TELEMETRY ---
            telemetry.addData("Heading angle (DEGREES)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("shooterVel", getAvgShooterVel());
            telemetry.addData("shooterTargetVel", shooterTargetVel);
            telemetry.addData("distanceToTag", getDistanceToTag());
            telemetry.addData("zone", closeZone ? "CLOSE" : farZone ? "FAR" : "NONE");
//            telemetry.addData("lShooterVelo", lShooter.getVelocity());
//            telemetry.addData("rShooterVelo", rShooter.getVelocity());
//            telemetry.addData("distanceSensor", distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    // =====================================================================
    // DRIVE METHODS
    // =====================================================================
    public void fieldCentric(double y, double x, double rx, double heading) {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * Constants.STRAFE_CORRECTION;

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

    public String autoAlign(double tolerance) {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "RESULT NULL";
        } else if (!result.isValid()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "RESULT INVALID";
        }

        double tx = result.getTx() - (getDistanceToTag() > 40 ? Constants.TX_OFFSET_DEGREES_CLOSE : Constants.TX_OFFSET_DEGREES_FAR);
        if (Math.abs(tx) <= tolerance) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "SUCCESS";
        }

        double pivot = Math.max(-1.0, Math.min(1.0, Constants.ALIGN_P_GAIN * tx));

        fieldCentric(0, 0, pivot, 0);

        return "IN PROGRESS";
    }

    // =====================================================================
    // SHOOTER METHODS
    // =====================================================================
    public void setShooterVelWithPower(double vel) {
        shooterTargetVel = vel;
        lShooter.setPower(shooterPID(vel, lShooter.getVelocity()));
        rShooter.setPower(shooterPID(vel, rShooter.getVelocity()));
    }

    public void setShooterVel(double vel) {
        shooterTargetVel = vel;
        lShooter.setVelocity(vel);
        rShooter.setVelocity(vel);
    }

    public int getAvgShooterVel() {
        return (int) ((lShooter.getVelocity() + rShooter.getVelocity()) / 2);
    }

    public boolean shooterWithinTolerance(double target) {
        return withinTolerance(getAvgShooterVel(), target, Constants.SHOOTER_TOLERANCE);
    }

    public boolean withinTolerance(double val, double target, double tol) {
        return Math.abs(target - val) <= tol;
    }

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

//    public boolean withinDistance() {
//        return distance.getDistance(DistanceUnit.INCH) < Constants.ACTIVATION_DISTANCE;
//    }

    // =====================================================================
    // LIMELIGHT METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (isLatestResultValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(Constants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (Constants.GOAL_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return 9999;
    }

    public boolean isLatestResultValid() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
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
         * Predicts shooter velocity from distance using a piecewise-linear lookup table.
         * - If distance is outside the table range, clamps to the nearest endpoint.
         * - If distance is invalid (NaN/Inf) or the table is invalid, returns CLOSE_ZONE_VELOCITY.
         */
        public double predict(double distance) {
            if (Double.isNaN(distance) || Double.isInfinite(distance)) return Constants.CLOSE_ZONE_VELOCITY;
            if (distances == null || velocities == null) return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length == 0 || velocities.length == 0) return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length != velocities.length) return Constants.CLOSE_ZONE_VELOCITY;
            if (distances.length == 1) return velocities[0];

            // Clamp out-of-range
            if (distance <= distances[0]) return velocities[0];
            int last = distances.length - 1;
            if (distance >= distances[last]) return velocities[last];

            // Find bracketing segment [i, i+1]
            for (int i = 0; i < last; i++) {
                double d0 = distances[i];
                double d1 = distances[i + 1];

                if (distance >= d0 && distance <= d1) {
                    double v0 = velocities[i];
                    double v1 = velocities[i + 1];
                    double denom = (d1 - d0);
                    if (Math.abs(denom) < 1e-9) return (v0 + v1) * 0.5;
                    double t = (distance - d0) / denom;
                    return v0 + t * (v1 - v0);
                }
            }

            // Shouldn't happen if distances[] is monotonic and clamped above, but keep a safe fallback.
            return velocities[last];
        }
    }

    void setPattern(RevBlinkinLedDriver.BlinkinPattern p) {
        pattern = p;
        blinkinLedDriver.setPattern(pattern);
    }

    void initializeHardware() {
        // --- Initialize Hardware ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        index = hardwareMap.get(DcMotorEx.class, "transfer");
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        index.setTargetPosition(0);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET, Constants.PINPOINT_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

//        distance = hardwareMap.get(DistanceSensor.class, "distance");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }
}
