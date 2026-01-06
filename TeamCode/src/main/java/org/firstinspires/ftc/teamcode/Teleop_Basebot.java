package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

import java.util.ArrayList;
import java.util.List;

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
        public static final double CLOSE_ZONE_VELOCITY = 1300;
        public static final double FAR_ZONE_VELOCITY = 1800;
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;

        // Drive
        public static final double PIVOT_MULTIPLIER = 0.8;
        public static final double STRAFE_CORRECTION = 1.1;
        public static final double AUTO_ALIGN_TOLERANCE = 0.5;
        public static final double ALIGN_P_GAIN = 0.025;

        // Intake
        public static final double INTAKE_POWER = 1.0;
        public static final double INTAKE_REVERSE_POWER = -0.7;
        public static final double TRIGGER_THRESHOLD = 0.2;

        // Index
        public static final int INDEX_STEP = 280;
        public static final double PASSIVE_INDEX_VELOCITY = 50;

        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;

        // Pinpoint
        public static final double PINPOINT_X_OFFSET = -107.31371;
        public static final double PINPOINT_Y_OFFSET = 0.0;

        // AutoShoot regression data
        public static final double[] DISTANCES = {};
        public static final double[] VELOCITIES = {};
        public static final int REGRESSION_DEGREE = 2;
    }

    // =====================================================================
    // INSTANCE VARIABLES
    // =====================================================================
    private ElapsedTime runtime = new ElapsedTime();
    double direction_x, direction_y, pivot, heading;
    double FLPower, FRPower, BLPower, BRPower;
    boolean doAutoIndex = false;


    // Hardware
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    DistanceSensor distance;

    // Gamepad state
    Gamepad gamepad;
//    Gamepad lastGamepad;

    // Zone selection
    boolean closeZone = true;
    boolean farZone = false;

    // PID state
    double shooterIntegral = 0;
    double shooterLastError = 0;

    // Velocity regression
    ShooterVelocityRegression regression;

    // =====================================================================
    // MAIN OPMODE
    // =====================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initializeHardware();

        // Initialize regression
        regression = new ShooterVelocityRegression(Constants.REGRESSION_DEGREE);
        regression.addDataPoints(Constants.DISTANCES, Constants.VELOCITIES);

        gamepad = new Gamepad();
//        lastGamepad = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();
        pinpoint.resetPosAndIMU();

        // =========================================================
        // MAIN LOOP
        // =========================================================
        while (opModeIsActive()) {
//            lastGamepad.copy(gamepad);
            pinpoint.update();
            gamepad.copy(gamepad1);

            // --- DRIVING ---
            direction_x = gamepad.left_stick_x;
            direction_y = -gamepad.left_stick_y;
            pivot = gamepad.right_stick_x * Constants.PIVOT_MULTIPLIER;
            heading = pinpoint.getHeading(AngleUnit.RADIANS);

            if (gamepad.cross) {
                telemetry.addData("Auto Align Status: ", autoAlign(Constants.AUTO_ALIGN_TOLERANCE));
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
            } else if (gamepad.dpadDownWasPressed()) {
                farZone = true;
                closeZone = false;
            }

            // --- SHOOTER ---
            if (gamepad.triangle) {
                double predictedVel = regression.predict(getDistanceToTag());
                setShooterVelWithPower(predictedVel);
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

            // --- INDEX ---
            if (gamepad.dpadRightWasPressed()) {
                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP);
                intake.setPower(1.0);
            } else if (gamepad.dpadLeftWasPressed()) {
                setIndexPos(index.getCurrentPosition() - Constants.INDEX_STEP);
                intake.setPower(-1.0);
            } else if (gamepad.squareWasPressed()) {
                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP * 3);
                intake.setPower(1.0);
            } else if (gamepad.circle) {

                index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                index.setPower(1);
                intake.setPower(1);
            } else {
                index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // --- INTAKE ---
            if (gamepad.right_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_POWER);
                if (distance.getDistance(DistanceUnit.INCH) > 2) {
                    index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    index.setVelocity(Constants.PASSIVE_INDEX_VELOCITY);
                } else {
                    index.setPower(0.0);
                }
            } else if (gamepad.left_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_REVERSE_POWER);
            } else {
                intake.setPower(0);
            }

            // --- AUTO INDEX ---
//            if (gamepad.touchpadWasPressed()) {
//                doAutoIndex = !doAutoIndex;
//            }
//            if (doAutoIndex && distance.getDistance(DistanceUnit.INCH) < 2) {
//                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP);
//                intake.setPower(1.0);
//            }

            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("shooterVel", (lShooter.getVelocity() + rShooter.getVelocity()) / 2);
            telemetry.addData("distanceToTag", getDistanceToTag());
            telemetry.addData("closeZone", closeZone);
            telemetry.addData("farZone", farZone);
            telemetry.addData("heading", heading);
            telemetry.addData("lShooterVelo", lShooter.getVelocity());
            telemetry.addData("rShooterVelo", rShooter.getVelocity());
            telemetry.addData("doAutoIndex", doAutoIndex);
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

        double tx = result.getTx();
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
        lShooter.setPower(shooterPID(vel, lShooter.getVelocity()));
        rShooter.setPower(shooterPID(vel, rShooter.getVelocity()));
    }

    public void setShooterVel(double vel) {
        lShooter.setVelocity(vel);
        rShooter.setVelocity(vel);
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

    // =====================================================================
    // LIMELIGHT METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(Constants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (Constants.GOAL_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return Double.POSITIVE_INFINITY;
    }

    // =====================================================================
    // SHOOTER VELOCITY REGRESSION
    // =====================================================================
    public static class ShooterVelocityRegression {

        public static class DataPoint {
            public final double distance;
            public final double velocity;

            public DataPoint(double distance, double velocity) {
                this.distance = distance;
                this.velocity = velocity;
            }
        }

        private final int degree;
        private final List<DataPoint> dataPoints;
        private double[] coefficients;
        private boolean isFitted;

        public ShooterVelocityRegression(int degree) {
            this.degree = Math.max(1, degree);
            this.dataPoints = new ArrayList<>();
            this.coefficients = null;
            this.isFitted = false;
        }

        public void addDataPoints(double[] distances, double[] velocities) {
            if (distances == null || velocities == null) return;
            if (distances.length != velocities.length) return;

            for (int i = 0; i < distances.length; i++) {
                dataPoints.add(new DataPoint(distances[i], velocities[i]));
            }
            if (!dataPoints.isEmpty()) fit();
        }

        public void fit() {
            if (dataPoints.isEmpty()) return;

            int n = dataPoints.size();
            int m = degree + 1;

            double[][] X = new double[n][m];
            double[] y = new double[n];

            for (int i = 0; i < n; i++) {
                DataPoint point = dataPoints.get(i);
                y[i] = point.velocity;
                for (int j = 0; j < m; j++) {
                    X[i][j] = Math.pow(point.distance, j);
                }
            }

            coefficients = solveLeastSquares(X, y);
            isFitted = true;
        }

        private double[] solveLeastSquares(double[][] X, double[] y) {
            int n = X.length;
            int m = X[0].length;

            double[][] XTX = new double[m][m];
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < m; j++) {
                    double sum = 0;
                    for (int k = 0; k < n; k++) sum += X[k][i] * X[k][j];
                    XTX[i][j] = sum;
                }
            }

            double[] XTy = new double[m];
            for (int i = 0; i < m; i++) {
                double sum = 0;
                for (int k = 0; k < n; k++) sum += X[k][i] * y[k];
                XTy[i] = sum;
            }

            return solveSystem(XTX, XTy);
        }

        private double[] solveSystem(double[][] A, double[] b) {
            int n = A.length;
            double[][] aug = new double[n][n + 1];

            for (int i = 0; i < n; i++) {
                System.arraycopy(A[i], 0, aug[i], 0, n);
                aug[i][n] = b[i];
            }

            for (int i = 0; i < n; i++) {
                int maxRow = i;
                for (int k = i + 1; k < n; k++) {
                    if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) maxRow = k;
                }
                double[] temp = aug[i]; aug[i] = aug[maxRow]; aug[maxRow] = temp;

                for (int k = i + 1; k < n; k++) {
                    if (Math.abs(aug[i][i]) < 1e-10) aug[i][i] = 1e-10;
                    double factor = aug[k][i] / aug[i][i];
                    for (int j = i; j < n + 1; j++) aug[k][j] -= factor * aug[i][j];
                }
            }

            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--) {
                x[i] = aug[i][n];
                for (int j = i + 1; j < n; j++) x[i] -= aug[i][j] * x[j];
                x[i] = Math.abs(aug[i][i]) < 1e-10 ? 0 : x[i] / aug[i][i];
            }
            return x;
        }

        public double predict(double distance) {
            if (!isFitted || coefficients == null) return Constants.CLOSE_ZONE_VELOCITY;

            double result = 0;
            for (int i = 0; i < coefficients.length; i++) {
                result += coefficients[i] * Math.pow(distance, i);
            }
            return result;
        }
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
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
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

        distance = hardwareMap.get(DistanceSensor.class, "distance");
    }
}
