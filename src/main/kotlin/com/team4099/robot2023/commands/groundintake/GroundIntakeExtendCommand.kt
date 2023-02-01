package com.team4099.robot2023.commands.groundintake

//
// class GroundIntakeExtendCommand(val groundIntake: GroundIntake) : CommandBase() {
//
//  lateinit var lowerIntakeArm: SequentialCommandGroup
//
//  init {
//    addRequirements(groundIntake)
//  }
//  override fun initialize() {
//    val desiredAngle = groundIntake.actualArmStates[GroundIntakeConstants.ArmStates.INTAKE]?.get()
//      ?: GroundIntakeConstants.ArmStates.INTAKE.position
//    lowerIntakeArm = SequentialCommandGroup(
//      groundIntake.rotateGroundIntakeToAngle(desiredAngle),
//      groundIntake.holdArmPosition()
//    )
//    lowerIntakeArm.initialize()
//  }
//
//  override fun execute() {
//    lowerIntakeArm.execute()
//    groundIntake.setRollerVoltage(GroundIntakeConstants.RollerStates.INTAKE.voltage)
//  }
//  override fun isFinished(): Boolean {
//    return false
//  }
// }
