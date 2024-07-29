
#include "extvar.h"
#include "servo.h"

/*   Servo external variables initialisation   */
void servoExtVariablesInit(SERVO_CONTROL *servo) {


		/*    Initialise external variables   */
		EXT_VAR extKp;
		EXT_VAR extKi;
		EXT_VAR extKd;
		EXT_VAR extSetPoint;
		EXT_VAR extError;
		EXT_VAR extLpfConstant;
		EXT_VAR extIntegrator;
		EXT_VAR extDifferentaitor;
		EXT_VAR extProprotinal;
		EXT_VAR extOutputLimited;

		EXT_VAR extMaxAcceleration;
		EXT_VAR extMaxVelocity;
		EXT_VAR extTrajectoryStatus;
		EXT_VAR extTrajectoryFollowing;
		EXT_VAR extFollowingThreshold;

		EXT_VAR extId;
		EXT_VAR extFirmwareVer;
		EXT_VAR extGoalPosition;
		EXT_VAR extTorqueEnable;
		EXT_VAR extMotionDirection;

		EXT_VAR extMaxMotorTemp;
		EXT_VAR extMaxIntTemp;
		EXT_VAR extMaxVoltage;
		EXT_VAR extMinVoltage;
		EXT_VAR extMaxCurrent;
		EXT_VAR extMaxPosition;
		EXT_VAR extMinPosition;

		EXT_VAR extVelocity;
		EXT_VAR extAcceleration;
		EXT_VAR extInMotion;
		EXT_VAR extMotionThreshold;
		EXT_VAR extAngle;

		EXT_VAR extIntTemp;
		EXT_VAR extIntTempIIRFilterCoefficient;

		EXT_VAR extMotorTemp;
		EXT_VAR extMotorTempIIRFilterCoefficient;

		EXT_VAR extBatteryVoltage;
		EXT_VAR extBatteryVoltageIIRFilterCoefficient;

		EXT_VAR extMotorCurrent;
		EXT_VAR extMotorCurrentIIRFilterCoefficient;

		EXT_VAR extDutyCycle;
		EXT_VAR extDirection;
		EXT_VAR extFrictionCompensation;

		EXT_VAR extIMUIIRFilterCoefficient;
		EXT_VAR extIMUCFilterCoefficient;
		EXT_VAR extAngleX;
		EXT_VAR extAngleY;


		/*    Assign values to external variables   */
		extKp.variable            								= &servo->PID.Kp;
		extKp.writePrivilege      								= 1;
		extKp.byteSize           								= 2;

		extKi.variable            								= &servo->PID.Ki;
		extKi.writePrivilege      								= 1;
		extKi.byteSize           								= 2;

		extKd.variable            								= &servo->PID.Kd;
		extKd.writePrivilege      								= 1;
		extKd.byteSize           								= 2;

		extSetPoint.variable      								= &servo->PID.setPoint;
		extSetPoint.writePrivilege 								= 0;
		extSetPoint.byteSize      								= 2;

		extError.variable         								= &servo->PID.Error;
		extError.writePrivilege   								= 0;
		extError.byteSize         								= 2;

		extLpfConstant.variable   								= &servo->PID.lpfConstant;
		extLpfConstant.writePrivilege 							= 1;
		extLpfConstant.byteSize   								= 2;

		extIntegrator.variable    								= &servo->PID.extIntegrator;
		extIntegrator.writePrivilege 							= 0;
		extIntegrator.byteSize    								= 2;

		extDifferentaitor.variable 								= &servo->PID.extDifferentaitor;
		extDifferentaitor.writePrivilege 						= 0;
		extDifferentaitor.byteSize 								= 2;

		extProprotinal.variable    								= &servo->PID.extProprotinal;
		extProprotinal.writePrivilege 							= 0;
		extProprotinal.byteSize   								= 2;

		extOutputLimited.variable  								= &servo->PID.outputLimited;
		extOutputLimited.writePrivilege 						= 0;
		extOutputLimited.byteSize  								= 2;

		// Continue with the remaining variables
		extMaxAcceleration.variable 							= &servo->profile.maxAcceleration;
		extMaxAcceleration.writePrivilege 						= 1;
		extMaxAcceleration.byteSize 							= 2;

		extMaxVelocity.variable     							= &servo->profile.maxVelocity;
		extMaxVelocity.writePrivilege 							= 1;
		extMaxVelocity.byteSize     							= 2;

		extTrajectoryStatus.variable 							= &servo->profile.trajectoryStatus;
		extTrajectoryStatus.writePrivilege 						= 0;
		extTrajectoryStatus.byteSize 							= 1;

		extTrajectoryFollowing.variable 						= &servo->profile.trajectoryFollowing;
		extTrajectoryFollowing.writePrivilege 					= 0;
		extTrajectoryFollowing.byteSize 						= 1;

		extFollowingThreshold.variable 							= &servo->profile.followingThreshold;
		extFollowingThreshold.writePrivilege 					= 1;
		extFollowingThreshold.byteSize 							= 2;

		extId.variable            								= &servo->id;
		extId.writePrivilege      								= 0;
		extId.byteSize           								= 1;

		extFirmwareVer.variable   								= &servo->firmwareVer;
		extFirmwareVer.writePrivilege 							= 0;
		extFirmwareVer.byteSize   								= 1;

		extGoalPosition.variable  								= &servo->goalPosition;
		extGoalPosition.writePrivilege 							= 1;
		extGoalPosition.byteSize  								= 2;

		extTorqueEnable.variable  								= &servo->torqueEnable;
		extTorqueEnable.writePrivilege 							= 0;
		extTorqueEnable.byteSize  								= 1;

		extMotionDirection.variable 							= &servo->motionDirection;
		extMotionDirection.writePrivilege 						= 0;
		extMotionDirection.byteSize 							= 1;

		extMaxMotorTemp.variable  								= &servo->maxMotorTemp;
		extMaxMotorTemp.writePrivilege 							= 1;
		extMaxMotorTemp.byteSize  								= 2;

		extMaxIntTemp.variable    								= &servo->maxIntTemp;
		extMaxIntTemp.writePrivilege 							= 1;
		extMaxIntTemp.byteSize    								= 2;

		extMaxVoltage.variable    								= &servo->maxVoltage;
		extMaxVoltage.writePrivilege 							= 1;
		extMaxVoltage.byteSize    								= 2;

		extMinVoltage.variable    								= &servo->minVoltage;
		extMinVoltage.writePrivilege 							= 1;
		extMinVoltage.byteSize    								= 2;

		extMaxCurrent.variable    								= &servo->maxCurrent;
		extMaxCurrent.writePrivilege 							= 1;
		extMaxCurrent.byteSize    								= 2;

		extMaxPosition.variable   								= &servo->profile.maxPosition;
		extMaxPosition.writePrivilege 							= 1;
		extMaxPosition.byteSize   								= 2;

		extMinPosition.variable   								= &servo->profile.minPosition;
		extMinPosition.writePrivilege 							= 1;
		extMinPosition.byteSize   								= 2;

		extVelocity.variable      								= &servo->motion.velocity;
		extVelocity.writePrivilege 								= 0;
		extVelocity.byteSize      								= 2;

		extAcceleration.variable  								= &servo->motion.acceleration;
		extAcceleration.writePrivilege 							= 0;
		extAcceleration.byteSize  								= 2;

		extInMotion.variable      								= &servo->motion.inMotion;
		extInMotion.writePrivilege 								= 0;
		extInMotion.byteSize      								= 1;

		extMotionThreshold.variable 							= &servo->motion.motionThreshold;
		extMotionThreshold.writePrivilege 						= 1;
		extMotionThreshold.byteSize 							= 2;

		extAngle.variable         								= &servo->encoder.angle;
		extAngle.writePrivilege    								= 0;
		extAngle.byteSize         								= 2;

		extIntTemp.variable       								= &servo->intTemp.converData;
		extIntTemp.writePrivilege  								= 0;
		extIntTemp.byteSize       								= 2;

		extIntTempIIRFilterCoefficient.variable 				= &servo->intTemp.extIIRFilterCoefficient;
		extIntTempIIRFilterCoefficient.writePrivilege 			= 1;
		extIntTempIIRFilterCoefficient.byteSize 				= 1;

		extMotorTemp.variable     								= &servo->motorTemp.converData;
		extMotorTemp.writePrivilege 							= 0;
		extMotorTemp.byteSize     								= 2;

		extMotorTempIIRFilterCoefficient.variable 				= &servo->motorTemp.extIIRFilterCoefficient;
		extMotorTempIIRFilterCoefficient.writePrivilege 		= 1;
		extMotorTempIIRFilterCoefficient.byteSize 				= 1;

		extBatteryVoltage.variable 								= &servo->batteryVoltage.converData;
		extBatteryVoltage.writePrivilege 						= 0;
		extBatteryVoltage.byteSize 								= 2;

		extBatteryVoltageIIRFilterCoefficient.variable 			= &servo->batteryVoltage.extIIRFilterCoefficient;
		extBatteryVoltageIIRFilterCoefficient.writePrivilege 	= 1;
		extBatteryVoltageIIRFilterCoefficient.byteSize 			= 1;

		extMotorCurrent.variable  								= &servo->motorCurrent.converData;
		extMotorCurrent.writePrivilege 							= 0;
		extMotorCurrent.byteSize  								= 2;

		extMotorCurrentIIRFilterCoefficient.variable 			= &servo->motorCurrent.extIIRFilterCoefficient;
		extMotorCurrentIIRFilterCoefficient.writePrivilege 		= 1;
		extMotorCurrentIIRFilterCoefficient.byteSize 			= 1;

		extDutyCycle.variable     								= &servo->motor.dutyCycle;
		extDutyCycle.writePrivilege 							= 0;
		extDutyCycle.byteSize     								= 2;

		extDirection.variable     								= &servo->motionDirection;
		extDirection.writePrivilege 							= 0;
		extDirection.byteSize     								= 1;

		extFrictionCompensation.variable 						= &servo->motor.frictionCompensation;
		extFrictionCompensation.writePrivilege 					= 1;
		extFrictionCompensation.byteSize 						= 1;

		extIMUIIRFilterCoefficient.variable 					= &servo->imu.extIIRFilterCoefficient;
		extIMUIIRFilterCoefficient.writePrivilege 				= 1;
		extIMUIIRFilterCoefficient.byteSize 					= 1;

		extIMUCFilterCoefficient.variable 						= &servo->imu.extCFilterCoefficient;
		extIMUCFilterCoefficient.writePrivilege 				= 1;
		extIMUCFilterCoefficient.byteSize 						= 1;

		extAngleX.variable        								= &servo->imu.extAngleX;
		extAngleX.writePrivilege   								= 0;
		extAngleX.byteSize        								= 2;

		extAngleY.variable        								= &servo->imu.extAngleY;
		extAngleY.writePrivilege   								= 0;
		extAngleY.byteSize        								= 2;


		/*    Add external variables to list  */
		servo->externalVariableList[0] = &extKp;
		servo->externalVariableList[1] = &extKi;
		servo->externalVariableList[2] = &extKd;
		servo->externalVariableList[3] = &extSetPoint;
		servo->externalVariableList[4] = &extError;
		servo->externalVariableList[5] = &extLpfConstant;
		servo->externalVariableList[6] = &extIntegrator;
		servo->externalVariableList[7] = &extDifferentaitor;
		servo->externalVariableList[8] = &extProprotinal;
		servo->externalVariableList[9] = &extOutputLimited;

		servo->externalVariableList[10] = &extMaxAcceleration;
		servo->externalVariableList[11] = &extMaxVelocity;
		servo->externalVariableList[12] = &extTrajectoryStatus;
		servo->externalVariableList[13] = &extTrajectoryFollowing;
		servo->externalVariableList[14] = &extFollowingThreshold;

		servo->externalVariableList[15] = &extId;
		servo->externalVariableList[16] = &extFirmwareVer;
		servo->externalVariableList[17] = &extGoalPosition;
		servo->externalVariableList[18] = &extTorqueEnable;
		servo->externalVariableList[19] = &extMotionDirection;

		servo->externalVariableList[20] = &extMaxMotorTemp;
		servo->externalVariableList[21] = &extMaxIntTemp;
		servo->externalVariableList[22] = &extMaxVoltage;
		servo->externalVariableList[23] = &extMinVoltage;
		servo->externalVariableList[24] = &extMaxCurrent;
		servo->externalVariableList[25] = &extMaxPosition;
		servo->externalVariableList[26] = &extMinPosition;

		servo->externalVariableList[27] = &extVelocity;
		servo->externalVariableList[28] = &extAcceleration;
		servo->externalVariableList[29] = &extInMotion;
		servo->externalVariableList[30] = &extMotionThreshold;
		servo->externalVariableList[31] = &extAngle;

		servo->externalVariableList[32] = &extIntTemp;
		servo->externalVariableList[33] = &extIntTempIIRFilterCoefficient;

		servo->externalVariableList[34] = &extMotorTemp;
		servo->externalVariableList[35] = &extMotorTempIIRFilterCoefficient;

		servo->externalVariableList[36] = &extBatteryVoltage;
		servo->externalVariableList[37] = &extBatteryVoltageIIRFilterCoefficient;

		servo->externalVariableList[38] = &extMotorCurrent;
		servo->externalVariableList[39] = &extMotorCurrentIIRFilterCoefficient;

		servo->externalVariableList[40] = &extDutyCycle;
		servo->externalVariableList[41] = &extDirection;
		servo->externalVariableList[42] = &extFrictionCompensation;

		servo->externalVariableList[43] = &extIMUIIRFilterCoefficient;
		servo->externalVariableList[44] = &extIMUCFilterCoefficient;
		servo->externalVariableList[45] = &extAngleX;
		servo->externalVariableList[46] = &extAngleY;


}

