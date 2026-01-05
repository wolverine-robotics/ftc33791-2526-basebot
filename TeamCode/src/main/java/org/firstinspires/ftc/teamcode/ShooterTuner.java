package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * ShooterTuner - OpMode for adjusting and testing shooter motor speeds.
 * 
 * Controls:
 *   DPAD UP    - Increase target velocity (+50)
 *   DPAD DOWN  - Decrease target velocity (-50)
 *   DPAD RIGHT - Large increase (+200)
 *   DPAD LEFT  - Large decrease (-200)
 *   
 *   TRIANGLE   - Toggle between adjusting BOTH, LEFT only, or RIGHT only motor
 *   
 *   RIGHT BUMPER - Start shooters at target velocity
 *   LEFT BUMPER  - Stop shooters
 *   
 *   CROSS      - Reset velocity to 0
 *   CIRCLE     - Reset velocity to 1000 (default)
 *   
 *   SQUARE     - Index forward
 *   RIGHT TRIGGER - Intake forward
 *   LEFT TRIGGER  - Intake reverse
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    // Shooter motors
    DcMotorEx lShooter, rShooter;
    
    // Intake and Index motors
    DcMotorEx intake, index;
    
    // Target velocities
    double lTargetVelocity = 0;
    double rTargetVelocity = 0;
    
    // Velocity adjustment amounts
    private static final double SMALL_STEP = 50;
    private static final double LARGE_STEP = 200;
    private static final double DEFAULT_VELOCITY = 1000;
    private static final double MAX_VELOCITY = 6000;
    
    // Adjustment mode: 0 = BOTH, 1 = LEFT only, 2 = RIGHT only
    int adjustMode = 0;
    String[] MODE_NAMES = {"BOTH", "LEFT ONLY", "RIGHT ONLY"};
    
    // Motor running state
    boolean shootersRunning = false;
    
    // Gamepad state for edge detection
    Gamepad currentGamepad;
    Gamepad previousGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Shooter Tuner...");
        telemetry.update();
        
        initializeShooters();
        
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();
        
        telemetry.addData("Status", "Ready - Press START");
        telemetry.addLine();
        telemetry.addData("Controls", "DPAD: Adjust velocity");
        telemetry.addData("", "TRIANGLE: Switch adjustment mode");
        telemetry.addData("", "R BUMPER: Start | L BUMPER: Stop");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update gamepad state
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            
            // --- MODE TOGGLE ---
            if (currentGamepad.triangleWasPressed()) {
                adjustMode = (adjustMode + 1) % 3;
            }
            
            // --- VELOCITY ADJUSTMENTS ---
            if (currentGamepad.dpadUpWasPressed()) {
                adjustVelocity(SMALL_STEP);
            }
            if (currentGamepad.dpadDownWasPressed()) {
                adjustVelocity(-SMALL_STEP);
            }
            if (currentGamepad.dpadRightWasPressed()) {
                adjustVelocity(LARGE_STEP);
            }
            if (currentGamepad.dpadLeftWasPressed()) {
                adjustVelocity(-LARGE_STEP);
            }
            
            // --- PRESET BUTTONS ---
            if (currentGamepad.crossWasPressed()) {
                // Reset to 0
                if (adjustMode == 0 || adjustMode == 1) lTargetVelocity = 0;
                if (adjustMode == 0 || adjustMode == 2) rTargetVelocity = 0;
            }
            if (currentGamepad.circleWasPressed()) {
                // Reset to default
                if (adjustMode == 0 || adjustMode == 1) lTargetVelocity = DEFAULT_VELOCITY;
                if (adjustMode == 0 || adjustMode == 2) rTargetVelocity = DEFAULT_VELOCITY;
            }
//            if (currentGamepad.squareWasPressed()) {
//                // Copy velocity between motors
//                if (adjustMode == 1) {
//                    // Copying right to left (in left-only mode)
//                    lTargetVelocity = rTargetVelocity;
//                } else if (adjustMode == 2) {
//                    // Copying left to right (in right-only mode)
//                    rTargetVelocity = lTargetVelocity;
//                } else {
//                    // In BOTH mode, average them
//                    double avg = (lTargetVelocity + rTargetVelocity) / 2;
//                    lTargetVelocity = avg;
//                    rTargetVelocity = avg;
//                }
//            }
            
            // --- SHOOTER START/STOP ---
            if (currentGamepad.rightBumperWasPressed()) {
                shootersRunning = true;
                lShooter.setVelocity(lTargetVelocity);
                rShooter.setVelocity(rTargetVelocity);
            }
            if (currentGamepad.leftBumperWasPressed()) {
                shootersRunning = false;
                lShooter.setPower(0);
                rShooter.setPower(0);
            }
            
            // Update motor velocities if running
            if (shootersRunning) {
                lShooter.setVelocity(lTargetVelocity);
                rShooter.setVelocity(rTargetVelocity);
            }
            
            // --- INDEX ---
            if (currentGamepad.squareWasPressed()) {
                setIndexPos(index.getCurrentPosition() + Constants.INDEX_STEP);
            }
            
            // --- INTAKE ---
            if (currentGamepad.right_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_POWER);
            } else if (currentGamepad.left_trigger > Constants.TRIGGER_THRESHOLD) {
                intake.setPower(Constants.INTAKE_REVERSE_POWER);
            } else {
                intake.setPower(0);
            }
            
            // --- TELEMETRY ---

            telemetry.addData("=== SHOOTER TUNER ===", "");
            telemetry.addLine();

            telemetry.addData("Distance To Tag", getDistanceToTag());
            telemetry.addLine();
            
            telemetry.addData("Adjustment Mode", MODE_NAMES[adjustMode]);
            telemetry.addData("Shooters", shootersRunning ? "RUNNING" : "STOPPED");
            telemetry.addLine();
            
            telemetry.addData("--- TARGET ---", "");
            telemetry.addData("Left Target", "%.0f", lTargetVelocity);
            telemetry.addData("Right Target", "%.0f", rTargetVelocity);
            telemetry.addLine();
            
            telemetry.addData("--- ACTUAL ---", "");
            telemetry.addData("Left Velocity", "%.1f", lShooter.getVelocity());
            telemetry.addData("Right Velocity", "%.1f", rShooter.getVelocity());
            telemetry.addLine();
            
            telemetry.addData("--- ERROR ---", "");
            telemetry.addData("Left Error", "%.1f", lTargetVelocity - lShooter.getVelocity());
            telemetry.addData("Right Error", "%.1f", rTargetVelocity - rShooter.getVelocity());
            telemetry.addLine();
            
            telemetry.addData("--- INTAKE/INDEX ---", "");
            telemetry.addData("Index Position", index.getCurrentPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addLine();
            
            telemetry.addData("--- CONTROLS ---", "");
            telemetry.addData("DPAD U/D", "+/- 50 | DPAD L/R: +/- 200");
            telemetry.addData("R BUMP", "Start | L BUMP: Stop");
            telemetry.addData("TRIANGLE", "Toggle Mode | CROSS: Zero | CIRCLE: 1000");
            telemetry.addData("SQUARE", "Index | RT: Intake | LT: Reverse");
            
            telemetry.update();
        }
        
        // Stop motors on exit
        lShooter.setPower(0);
        rShooter.setPower(0);
    }
    
    private void adjustVelocity(double delta) {
        if (adjustMode == 0 || adjustMode == 1) {
            lTargetVelocity = clamp(lTargetVelocity + delta, 0, MAX_VELOCITY);
        }
        if (adjustMode == 0 || adjustMode == 2) {
            rTargetVelocity = clamp(rTargetVelocity + delta, 0, MAX_VELOCITY);
        }
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }
    
    private void initializeShooters() {
        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Index
        index = hardwareMap.get(DcMotorEx.class, "transfer");
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        index.setTargetPosition(0);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public static class Constants {
        // Shooter
        public static final double CLOSE_ZONE_VELOCITY = 1000;
        public static final double FAR_ZONE_VELOCITY = 1200;
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

    Limelight3A limelight;



    // =====================================================================
    // LIMELIGHT METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(Teleop_Basebot.Constants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (Teleop_Basebot.Constants.GOAL_HEIGHT - Teleop_Basebot.Constants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return Double.POSITIVE_INFINITY;
    }
}

