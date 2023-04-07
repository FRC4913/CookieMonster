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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    // region DEFINE VARIABLES
    HuskyBot huskyBot = new HuskyBot();

    private ElapsedTime runtime = new ElapsedTime();
    final double END_GAME_TIME = 80.0;  // last 40 seconds
    final double FINAL_TIME = 110.0;    // last 10 seconds
    boolean endGameRumbled = false;
    boolean finalRumbled = false;


    // endregion

    // region DEFINE FUNCTIONS


    @Override
    public void runOpMode() {

        // region INITIALIZATION
        huskyBot.init(hardwareMap);
        huskyBot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        huskyBot.arm.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.arm.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
        // endregion


        // region TELE-OP LOOP
        while (opModeIsActive()) {

        // region CONTROLLER RUMBLE SIGNALS
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled) {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled) {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }
        // endregion


        // region DRIVE MECHANISMS
            Pose2d poseEstimate = huskyBot.drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());
            // uses the left trigger to dynamically shift between different drive speeds.
            // when the trigger is fully released, driveVelocity = 1.
            // when the trigger is fully pressed, driveVelocity = 0.2.
            double driveVelocity = (0.35 + 0.5 * gamepad1.left_trigger);

            huskyBot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * driveVelocity,
                            input.getY() * driveVelocity,
                            -gamepad1.right_stick_x * driveVelocity
                    )
            );
            huskyBot.drive.update();
        // endregion


        // region MANUAL ARM CONTROL
            // region Blocked by Preset FSM
            if (armState == ArmState.ARM_WAIT) {
                // org.firstinspires.ftc.teamcode.Arm Lift Controls
                armLiftTargetPos += 30 * -gamepad2.left_stick_y;
                if (armLiftTargetPos > ARM_LIFT_MAX_POSITION) {
                    armLiftTargetPos = ARM_LIFT_MAX_POSITION;
                }
                if (armLiftTargetPos < ARM_LIFT_MIN_POSITION) {
                    armLiftTargetPos = ARM_LIFT_MIN_POSITION;
                }
                huskyBot.arm.armLiftRunToPos(armLiftTargetPos);

                // Increases/Decreases Arm Length
                armExtendPower = gamepad2.dpad_up ? -ARM_EXTENSION_MAX_POWER : (gamepad2.dpad_down ? ARM_EXTENSION_MAX_POWER : 0);
                // Use Magnetic Limit Switches to limit extension of the arm.
                if (huskyBot.arm.armExtendMin.isPressed()) {
                    armExtendPower = (armExtendPower > 0) ? 0 : armExtendPower;
                }
                if (huskyBot.arm.armExtendMax.isPressed()) {
                    armExtendPower = (armExtendPower < 0) ? 0 : armExtendPower;
                }
                huskyBot.arm.armExtendMotor.setPower(armExtendPower);


                // Claw Lift Servo Control
                if (gamepad2.right_stick_y != 0) {
                    huskyBot.arm.servoMove(huskyBot.arm.clawLift, -gamepad2.right_stick_y);
                }
            }
            // endregion

            // region Not Blocked by Preset FSM
            // Arm Swivel Controls
            armSwivelPower = -gamepad2.left_stick_x;
            armSwivelPower = Range.clip(armSwivelPower, -ARM_SWIVEL_MAX_POWER, ARM_SWIVEL_MAX_POWER);
            // Arm Swivel Limiters
            if (huskyBot.arm.armSwivelMotor.getCurrentPosition() <= ARM_SWIVEL_RIGHT_LIMIT) {
                armSwivelPower = (armSwivelPower < 0) ? 0 : armSwivelPower;
            }
            if (huskyBot.arm.armSwivelMotor.getCurrentPosition() >= ARM_SWIVEL_LEFT_LIMIT) {
                armSwivelPower = (armSwivelPower > 0) ? 0 : armSwivelPower;
            }
            huskyBot.arm.armSwivelMotor.setPower(armSwivelPower);


            // Open/Close the Claw
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                huskyBot.arm.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);
            }
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                huskyBot.arm.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
            }

            // Custom Claw Open/Close
            if (-gamepad2.right_trigger != 0) {
                huskyBot.arm.servoMove(huskyBot.arm.clawGrab, -gamepad2.right_trigger);
            }
            if (gamepad2.left_trigger != 0) {
                huskyBot.arm.servoMove(huskyBot.arm.clawGrab, gamepad2.left_trigger);
            }
            // endregion
        // endregion


        // region ARM PRESETS FINITE STATE MACHINE
            if(armState != ArmState.ARM_WAIT && (gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad2.left_stick_button || gamepad2.right_stick_button)){
                huskyBot.arm.armExtendMotor.setPower(0);
                huskyBot.arm.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                armLiftTargetPos = huskyBot.arm.armLiftMotor.getCurrentPosition();

                armState = ArmState.ARM_WAIT;
            }

            if(gamepad1.a) {
                // Button A: Ground position
                huskyBot.arm.setPositionToGroundJunction();
            }
            if(gamepad1.x){
                // Button X: Low junction position
                huskyBot.arm.setPositionToLowJunction();
            }
            if(gamepad1.b) {
                // Button B: Medium junction position
                huskyBot.arm.setPositionToMediumJunction();
            }
            if(gamepad1.y) {
                // Button Y: High junction position
                huskyBot.arm.setPositionToHighJunction();
            }

            huskyBot.arm.run();

        // endregion


        // region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Drive Mechanism Telemetry
            telemetry.addData("Stick", "y (%.2f), x (%.2f), rx (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Heading", poseEstimate.getHeading());

            // Arm Telemetry
            telemetry.addData("Arm Swivel", "Power: (%.2f), Pos: (%d)", huskyBot.arm.armSwivelMotor.getPower(), huskyBot.arm.armSwivelMotor.getCurrentPosition());
            telemetry.addData("Arm Lift", "Left Y: (%.2f), Power: (%.2f), Pos: (%d)", gamepad2.left_stick_y, huskyBot.arm.armLiftMotor.getPower(), huskyBot.arm.armLiftMotor.getCurrentPosition());
            telemetry.addData("Arm Extend", "Power: (%.2f), Pos: (%d)", huskyBot.arm.armExtendMotor.getPower(), huskyBot.arm.armExtendMotor.getCurrentPosition());

            // Claw Telemetry
            telemetry.addData("Claw Lift", "Right Y: (%.2f), Pos: (%.2f)",gamepad2.right_stick_y, huskyBot.arm.clawLift.getPosition());
            telemetry.addData("Claw Grab", "Pos: (%.2f)", huskyBot.arm.clawGrab.getPosition());

            telemetry.addData("current pos ", huskyBot.arm.armLiftMotor.getCurrentPosition());
            telemetry.addData("target pos ", armLiftTargetPos);
            telemetry.addData("follow point ", follow_point);
            telemetry.addData("vel ", huskyBot.arm.armLiftMotor.getVelocity());

            telemetry.update();
        // endregion

        }
        // endregion

    }
}

