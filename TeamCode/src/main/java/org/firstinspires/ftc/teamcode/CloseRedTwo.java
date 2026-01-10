package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "CloseRedTwo", group = "AUTO")
public class CloseRedTwo extends LinearOpMode {

    // ===================== CONSTANTS =====================
    static final double DRIVE_kP = 0.035;
    static final double STRAFE_kP = 0.04;

    static final double BACKUP_DISTANCE = -28; // inches (negative = backward)
    static final double EXIT_FORWARD = 0.75;     // slightly forward toward goal
    static final double EXIT_STRAFE = 16.0;      // right (positive Y)

    static final double SHOOTER_VELOCITY = 955;
    static final double SHOOTER_VELOCITYTWO = 1125;
    static final double MAG_POWER = -0.9;

    // ===================== HARDWARE =====================
    DcMotorEx fl, fr, bl, br;
    DcMotorEx lShooter, rShooter, index, intake;

    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;

    // ===================== OPMODE =====================
    @Override
    public void runOpMode() {

        // Drive
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter
        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);


        // Coast to prevent motor damage
        lShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        index = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Sensors
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(
                Teleop_Basebot.Constants.PINPOINT_X_OFFSET,
                Teleop_Basebot.Constants.PINPOINT_Y_OFFSET,
                DistanceUnit.MM
        );
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        // ===================== PINPOINT RESET (CRITICAL) =====================
        pinpoint.resetPosAndIMU();
        sleep(200);

        // ===================== AUTO SEQUENCE =====================

        // 1️⃣ Back away from goal (~24 inches)
        driveToX(BACKUP_DISTANCE, 1.605);

        // 2️⃣ Auto-align to goal
        //   autoAlign(0.5, 2.0);
       // autoAlignWithTimeout(0.5, 1.0);

        // 3️⃣ Spin up shooter
        lShooter.setVelocity(SHOOTER_VELOCITY);
        rShooter.setVelocity(SHOOTER_VELOCITY);
        sleep(850);

        // 4️⃣ Dump all balls (extra time for 3rd)
        index.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        index.setPower(0.65);
        intake.setPower(-0.95);
        sleep(1600);
        index.setPower(0);
//        intake.setPower(0);

        // 5️⃣ Exit: move slightly forward, then strafe right
        turnToAngle(-45, 2);
        pinpoint.setHeading(180, AngleUnit.DEGREES);
        sleep(300);
        telemetry.addData("PinpointPose:",pinpoint.getPosition().toString());
        telemetry.update();
        driveToX(41, 3.0);
      pinpoint.setHeading(180, AngleUnit.DEGREES);
        sleep(500);
        driveToX(-41, 3.0);
        
      turnToAngle(45, 2);
        

        // 3️⃣ Spin up shooter
       lShooter.setVelocity(SHOOTER_VELOCITYTWO);
        rShooter.setVelocity(SHOOTER_VELOCITYTWO);
         sleep(1050);

        // 4️⃣ Dump all balls (extra time for 3rd)
        index.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        index.setPower(0.65);
        intake.setPower(-0.95);
        sleep(1600);
        index.setPower(0);
        intake.setPower(0);
        
        turnToAngle(-45, 2);
        pinpoint.setHeading(180, AngleUnit.DEGREES);
        sleep(300);
        telemetry.addData("PinpointPose:",pinpoint.getPosition().toString());
        telemetry.update();
        driveToX(25, 3.0);

        stopDrive();
    }

    // ===================== ALIGN =====================
    void autoAlignWithTimeout(double tolerance, double timeout) {
        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < timeout) {
            LLResult r = limelight.getLatestResult();
            if (r == null || !r.isValid()) continue;

            double tx = r.getTx();
            if (Math.abs(tx) <= tolerance) break;

            double turn = Teleop_Basebot.Constants.ALIGN_P_GAIN * tx;
            setDrive(0, 0, turn);
        }
        stopDrive();
    }

    // ===================== DRIVE HELPERS =====================

    void driveToX(double deltaInches, double timeoutSec) {
        
        ElapsedTime t = new ElapsedTime();
        pinpoint.update();
        double targetX = pinpoint.getPosX(DistanceUnit.INCH) + deltaInches;

        while (opModeIsActive() && t.seconds() < timeoutSec) {
            pinpoint.update();
            double error = targetX - pinpoint.getPosX(DistanceUnit.INCH);

            if (Math.abs(error) < 0.75) break;

            double power = DRIVE_kP * error;
            power = clamp(power, -0.4, 0.4);

            setDrive(power, 0, 0);
        }
        stopDrive();
    }

    void driveToY(double deltaInches, double timeoutSec) {
        ElapsedTime t = new ElapsedTime();
        pinpoint.update();
        double targetY = pinpoint.getPosY(DistanceUnit.INCH) + deltaInches;

        while (opModeIsActive() && t.seconds() < timeoutSec) {
            pinpoint.update();
            double error = targetY - pinpoint.getPosY(DistanceUnit.INCH);

            if (Math.abs(error) < 0.75) break;

            double power = STRAFE_kP * error;
            power = clamp(power, -0.4, 0.4);

            setDrive(0, power, 0);
        }
        stopDrive();
    }

    void turnToAngle(double deltaAngle, double timeoutSec) {
        ElapsedTime t = new ElapsedTime();
        pinpoint.update();
        double targetAngle = AngleUnit.normalizeDegrees(pinpoint.getHeading(AngleUnit.DEGREES)+deltaAngle);

        while (opModeIsActive() && t.seconds() < timeoutSec) {
            pinpoint.update();
            double error = AngleUnit.normalizeDegrees(targetAngle - pinpoint.getHeading(AngleUnit.DEGREES));

            if (Math.abs(error) < 0.75) break;

            double power = 0.025 * error;
            power = clamp(power, -0.4, 0.4);

            setDrive(0, 0, -power);
        }
        stopDrive();
    }

    void setDrive(double x, double y, double rx) {
        double flp = x + y + rx;
        double frp = x - y - rx;
        double blp = x - y + rx;
        double brp = x + y - rx;

        double max = Math.max(1,
                Math.max(Math.abs(flp),
                        Math.max(Math.abs(frp),
                                Math.max(Math.abs(blp), Math.abs(brp)))));

        fl.setPower(flp / max);
        fr.setPower(frp / max);
        bl.setPower(blp / max);
        br.setPower(brp / max);
    }

    void stopDrive() {
        setDrive(0, 0, 0);
    }

    double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
