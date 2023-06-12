package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Arm {
    // org.firstinspires.ftc.teamcode.Arm Control Motor Init.
    public DcMotorEx armSwivelMotor = null;
    public DcMotorEx armLiftMotor = null;
    public DcMotorEx armExtendMotor = null;




    // Claw Servo Init.
    public Servo clawLift = null;
    public Servo clawGrab = null;

    // Magnetic Limit Switches
    public TouchSensor armExtendMax = null;
    public TouchSensor armExtendMin = null;


    // org.firstinspires.ftc.teamcode.Arm Lift PID Controller Init.
    public static final double up_Kp = 0.0055, up_Ki = 0, up_Kd = 0.00025;
    public static final double down_Kp = 0.00038, down_Ki = 0, down_Kd = 0.00001;
    public static PIDController armUpPID = new PIDController(up_Kp, up_Ki, up_Kd);
    public static PIDController armDownPID = new PIDController(down_Kp, down_Ki, down_Kd);
    public static final double f = 0.045, l = 0;
    public static final double ARM_LIFT_TICKS_PER_DEGREE = 28.0 * 5.23 * 5.23 * 3.61 / 360;


    // Define org.firstinspires.ftc.teamcode.Arm Constants: Power
    public static final double ARM_SWIVEL_MAX_POWER = 0.35;
    public static final double ARM_LIFT_MAX_POWER = 0.5;
    public static final double ARM_LIFT_MIN_POWER = 0.01;
    public static final double ARM_LIFT_POWER_AT_REST = 0.12;
    public static final double ARM_EXTENSION_MAX_POWER = 0.6;

    // Define org.firstinspires.ftc.teamcode.Arm Constants: Encoder and Servo Values
    public static final int ARM_SWIVEL_RIGHT_LIMIT = -70;
    public static final int ARM_SWIVEL_LEFT_LIMIT = 160;

    public static final int ARM_LIFT_MAX_POSITION = 910;
    public static final int ARM_LIFT_MIN_POSITION = 20;

    public static final double CLAW_MOVE_INCREMENT = 0.025;
    public static final double CLAW_LIFT_MIN_RANGE = 0.0;
    public static final double CLAW_LIFT_MAX_RANGE = 1.0;
    public static final double CLAW_LIFT_START_POSITION = 0.6;   // scaled, see MIN and MAX_RANGE

    public static final double CLAW_GRAB_MIN_RANGE = 0.1;
    public static final double CLAW_GRAB_MAX_RANGE = 0.54;
    public static final double CLAW_GRAB_OPEN_POSITION = 0.3;
    public static final double CLAW_GRAB_CLOSE_POSITION = 1.0;

    // Define org.firstinspires.ftc.teamcode.Arm Constants: Preset Junction Positions
    public static final int ARM_LIFT_GROUND_POSITION = 20, ARM_EXTEND_GROUND_POSITION = -1825; public static final double CLAW_LIFT_GROUND_POSITION = 0.55;
    public static final int ARM_LIFT_LOW_POSITION = 360, ARM_EXTEND_LOW_POSITION = -10; public static final double CLAW_LIFT_LOW_POSITION = 0.55;
    public static final int ARM_LIFT_MED_POSITION = 660, ARM_EXTEND_MED_POSITION = -10; public static final double CLAW_LIFT_MED_POSITION = 0.50;
    public static final int ARM_LIFT_HIGH_POSITION = 880, ARM_EXTEND_HIGH_POSITION = -3215; public static final double CLAW_LIFT_HIGH_POSITION = 0.35;


    public static double armSwivelPower = 0.0;
    public static double armExtendPower = 0.0;
    public static double armLiftPower = 0.0;

    private final ElapsedTime finiteTimer = new ElapsedTime();
    public enum ArmState {
        ARM_WAIT,
        STEP_1,
        STEP_2,
        STEP_3,
        STEP_4
    }
    public static ArmState armState = ArmState.ARM_WAIT;
    public static int follow_point = ARM_LIFT_MIN_POSITION;

    public static int armLiftTargetPos = 0;
    public static int armExtendTargetPos;
    public static double clawLiftTargetPos;
    public static boolean shouldChangeTheClawLift = false;

    public Arm(HardwareMap ahwMap) {
        // Define and Initialize org.firstinspires.ftc.teamcode.Arm Motors and Servos
        armSwivelMotor = ahwMap.get(DcMotorEx.class, "arm_swivel");
        armLiftMotor = ahwMap.get(DcMotorEx.class, "arm_lift");
        armExtendMotor = ahwMap.get(DcMotorEx.class, "arm_extend");

        clawLift = ahwMap.get(Servo.class, "claw_lift");
        clawGrab = ahwMap.get(Servo.class, "claw_grab");
        clawLift.scaleRange(CLAW_LIFT_MIN_RANGE, CLAW_LIFT_MAX_RANGE);
        clawGrab.scaleRange(CLAW_GRAB_MIN_RANGE, CLAW_GRAB_MAX_RANGE);

        // Define and Init. Magnetic Limit Switches
        armExtendMax = ahwMap.get(TouchSensor.class, "arm_extend_max");
        armExtendMin = ahwMap.get(TouchSensor.class, "arm_extend_min");

        // Set org.firstinspires.ftc.teamcode.Arm Motors to Zero Power
        armSwivelMotor.setPower(0);
        armLiftMotor.setPower(0);
        armExtendMotor.setPower(0);

        // Reset org.firstinspires.ftc.teamcode.Arm Motor Encoders
        armSwivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set org.firstinspires.ftc.teamcode.Arm Motor Behaviors
        armSwivelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armSwivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void servoMove(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        if (targetPosition > 0) {
            servo.setPosition(currentPosition + CLAW_MOVE_INCREMENT);
        }
        else if (targetPosition < 0) {
            servo.setPosition(currentPosition - CLAW_MOVE_INCREMENT);
        }
    }

    // method to move the arm lift to a given position using PID control.
    void armLiftRunToPos(int target) {
        int armPos = armLiftMotor.getCurrentPosition();
        double pid;
        if (target >= armPos) {
            pid = armUpPID.calculate(armPos, target);
        } else {
            pid = armDownPID.calculate(armPos, target);
        }

        int extendPos = armExtendMotor.getCurrentPosition();
        double targetArmAngle = Math.toRadians((target - 470) / ARM_LIFT_TICKS_PER_DEGREE);
        double ff = (l * extendPos + 1) * f * Math.cos(targetArmAngle);

        armLiftPower = pid + ff;
        armLiftMotor.setPower(armLiftPower);
    }
    // endregion

    public void run(LinearOpMode opMode){
        new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                switch (armState){
                    case ARM_WAIT:
                        finiteTimer.reset();
                        shouldChangeTheClawLift = false;
                        break;
                    case STEP_1:
                        // Step 1: Reset the arm extender (close)

                        armExtendMotor.setTargetPosition(-10);
                        armExtendMotor.setPower(1.0);
                        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        armState = ArmState.STEP_2;

                        break;

                    case STEP_2:
                        // Step 2:
                        // Wait until the step 1 is completed
                        // Then change the arm lift's position (up or down based on the target position)
                        armLiftRunToPos(armLiftMotor.getCurrentPosition());

                        if(armExtendMotor.isBusy()){
                            if(finiteTimer.seconds() > 7){
                                armExtendMotor.setPower(0);
                                armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                                armLiftTargetPos = armLiftMotor.getCurrentPosition();

                                armState = ArmState.ARM_WAIT;
                                break;
                            }
                        } else {
                            armExtendMotor.setPower(0);
                            armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            follow_point = armLiftMotor.getCurrentPosition();

                            armState = ArmState.STEP_3;
                        }

                        break;
                    case STEP_3:
                        // Step 3:
                        // Wait until the step 2 is completed
                        // Then change the arm extender's position (in or out based on the target position)

                        // Gradually increment follow_point until it reaches the target position.
                        int error = follow_point - armLiftTargetPos;
                        if (error > 50) {
                            follow_point -= 50;
                        } else if (error < 50) {
                            follow_point += 50;
                        } else {
                            follow_point = armLiftTargetPos;
                        }
                        armLiftRunToPos(follow_point);

                        if (Math.abs(armLiftMotor.getCurrentPosition() - armLiftTargetPos) > 50) {
                            if(finiteTimer.seconds() > 7){
                                armLiftTargetPos = armLiftMotor.getCurrentPosition();

                                armState = ArmState.ARM_WAIT;
                                break;
                            }

                        } else {
                            armExtendMotor.setTargetPosition(armExtendTargetPos);
                            armExtendMotor.setPower(1.0);
                            armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            armState = ArmState.STEP_4;
                        }


                        break;
                    case STEP_4:
                        // Step 4:
                        // Wait until the step 3 is completed
                        // Then change armState to wait (default)
                        armLiftRunToPos(armLiftTargetPos);

                        if (armExtendMotor.isBusy()) {
                            if(finiteTimer.seconds() > 7){
                                armExtendMotor.setPower(0);
                                armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                                armLiftTargetPos = armLiftMotor.getCurrentPosition();

                                armState = ArmState.ARM_WAIT;
                                break;
                            }
                        } else {
                            armExtendMotor.setPower(0);
                            armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            if(shouldChangeTheClawLift){
                                if(finiteTimer.seconds() > 7){
                                    armState = ArmState.ARM_WAIT;
                                    break;
                                }

                                clawLift.setPosition(clawLiftTargetPos);
                            }

                            armState = ArmState.ARM_WAIT;
                        }

                        break;

                    default:
                }
            }
        }).start();

    }

    public void setPositionToGroundJunction(){
        clawLift.setPosition(CLAW_LIFT_GROUND_POSITION);
        clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);

        armLiftTargetPos = ARM_LIFT_GROUND_POSITION;
        armExtendTargetPos = ARM_EXTEND_GROUND_POSITION;

        armState = ArmState.STEP_1;
    }

    public void setPositionToLowJunction(){
        armLiftTargetPos = ARM_LIFT_LOW_POSITION;
        armExtendTargetPos = ARM_EXTEND_LOW_POSITION;
        clawLiftTargetPos = CLAW_LIFT_LOW_POSITION;

        shouldChangeTheClawLift = true;

        armState = ArmState.STEP_1;
    }

    public void setPositionToMediumJunction(){
        armLiftTargetPos = ARM_LIFT_MED_POSITION;
        armExtendTargetPos = ARM_EXTEND_MED_POSITION;
        clawLiftTargetPos = CLAW_LIFT_MED_POSITION;

        shouldChangeTheClawLift = true;

        armState = ArmState.STEP_1;
    }

    public void setPositionToHighJunction(){
        armLiftTargetPos = ARM_LIFT_HIGH_POSITION;
        armExtendTargetPos = ARM_EXTEND_HIGH_POSITION;
        clawLiftTargetPos = CLAW_LIFT_HIGH_POSITION;

        shouldChangeTheClawLift = true;

        armState = ArmState.STEP_1;
    }
}
