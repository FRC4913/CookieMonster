/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.common.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for the robot.
 * <p>
 */
@Config
public class HuskyBot {
    /* Public OpMode members. */
    public SampleMecanumDrive drive = null;

    public DcMotorEx frontLeftDrive = null;
    public DcMotorEx frontRightDrive = null;
    public DcMotorEx rearLeftDrive = null;
    public DcMotorEx rearRightDrive = null;

    // Arm Control Motor Init.
    public DcMotorEx armSwivelMotor = null;
    public DcMotorEx armLiftMotor = null;
    public DcMotorEx armExtendMotor = null;


    // Claw Servo Init.
    public Servo clawLift = null;
    public Servo clawGrab = null;

    // Webcam
    public OpenCvWebcam webcam;

    // Magnetic Limit Switches
    public TouchSensor armExtendMax = null;
    public TouchSensor armExtendMin = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    public HuskyBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Mecanum Drive
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Arm Motors and Servos
        armSwivelMotor = hwMap.get(DcMotorEx.class, "arm_swivel");
        armLiftMotor = hwMap.get(DcMotorEx.class, "arm_lift");
        armExtendMotor = hwMap.get(DcMotorEx.class, "arm_extend");

        clawLift = hwMap.get(Servo.class, "claw_lift");
        clawGrab = hwMap.get(Servo.class, "claw_grab");
        clawLift.scaleRange(CLAW_LIFT_MIN_RANGE, CLAW_LIFT_MAX_RANGE);
        clawGrab.scaleRange(CLAW_GRAB_MIN_RANGE, CLAW_GRAB_MAX_RANGE);

        // Define and Init. Magnetic Limit Switches
        armExtendMax = hwMap.get(TouchSensor.class, "arm_extend_max");
        armExtendMin = hwMap.get(TouchSensor.class, "arm_extend_min");

        // Set Arm Motors to Zero Power
        armSwivelMotor.setPower(0);
        armLiftMotor.setPower(0);
        armExtendMotor.setPower(0);

        // Reset Arm Motor Encoders
        armSwivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Arm Motor Behaviors
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
}
