package com.team4099.robot2023.subsystems.gameboy

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.config.constants.Substation
import com.team4099.robot2023.subsystems.gameboy.objective.Objective
import com.team4099.robot2023.util.FMSData
import edu.wpi.first.networktables.IntegerPublisher
import edu.wpi.first.networktables.IntegerSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import io.javalin.Javalin
import io.javalin.http.staticfiles.Location
import java.nio.file.Paths

object GameboyIOServer : GameboyIO {
  private var nodePublisher: IntegerPublisher? = null
  private var nodeSubscriber: IntegerSubscriber? = null
  private var coneTippedPublisher: IntegerPublisher? = null
  private var coneTippedSubscriber: IntegerSubscriber? = null

  init {
    // Create publisher and subscriber
    val table = NetworkTableInstance.getDefault().getTable("nodeselector")
    nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish()
    nodeSubscriber =
      table
        .getIntegerTopic("node_dashboard_to_robot")
        .subscribe(-1) // needs to be -1 so nothing is displayed
    coneTippedPublisher = table.getIntegerTopic("substation_selection_robot_to_dashboard").publish()
    coneTippedSubscriber =
      table
        .getIntegerTopic("substation_selection_dashboard_to_robot")
        .subscribe(-1) // needs to be -1 so nothing is displayed

    // Start server
    val app =
      Javalin.create { config ->
        config.staticFiles.add(
          Paths.get(Filesystem.getDeployDirectory().absolutePath.toString(), "nodeselector")
            .toString(),
          Location.EXTERNAL
        )
      }
    app.start(5800)
  }

  override fun updateInputs(inputs: GameboyIO.GameboyIOInputs) {

    val objective = inputs.objective

    for (value in nodeSubscriber!!.readQueueValues()) {
      if (value.toInt() != -1) {
        val selectedNode = positionIndexToSelectedNode(value.toInt())
        objective.nodeColumn = selectedNode.first
        objective.nodeTier = selectedNode.second
      }
    }
    for (value in coneTippedSubscriber!!.readQueueValues()) {
      if (value.toInt() != -1) {
        objective.substation = substationIndexToObjective(value.toInt())
      }
    }

    inputs.objective = objective
  }

  override fun setSelected(objective: Objective) {

    var selected = 0
    if (FMSData.allianceColor == DriverStation.Alliance.Blue) {
      selected += (8 - objective.nodeColumn)
    } else {
      selected += objective.nodeColumn
    }
    when (objective.nodeTier) {
      Constants.Universal.NodeTier.HYBRID -> selected += 0
      Constants.Universal.NodeTier.MID -> selected += 9
      Constants.Universal.NodeTier.HIGH -> selected += 18
      else -> selected += 0
    }

    nodePublisher!!.set(selected.toLong())
    setConeOrientation(objective.substation)
  }

  fun setConeOrientation(substation: Substation) {
    val index =
      if (substation == Substation.SINGLE_SUBSTATION) 0
      else if (substation == Constants.Universal.Substation.DOUBLE_SUBSTATION_LEFT) 1
      else if (substation == Substation.DOUBLE_SUBSTATION_RIGHT) 2 else -1
    coneTippedPublisher!!.set(index.toLong())
  }

  private fun positionIndexToSelectedNode(position: Int): Pair<Int, NodeTier> {
    var column: Int
    var nodeLevel: Constants.Universal.NodeTier
    if (FMSData.allianceColor == DriverStation.Alliance.Blue) {
      column = 8 - position % 9
    } else {
      column = position % 9
    }

    if (position < 9) {
      nodeLevel = Constants.Universal.NodeTier.HYBRID
    } else if (position < 18) {
      nodeLevel = Constants.Universal.NodeTier.MID
    } else {
      nodeLevel = Constants.Universal.NodeTier.HIGH
    }

    return Pair(column, nodeLevel)
  }

  private fun substationIndexToObjective(index: Int): Substation {
    val substation =
      if (index == 0) {
        Substation.SINGLE_SUBSTATION
      } else if (index == 1) {
        Substation.DOUBLE_SUBSTATION_LEFT
      } else if (index == 2) {
        Substation.DOUBLE_SUBSTATION_RIGHT
      } else Substation.NONE

    return substation
  }
}
