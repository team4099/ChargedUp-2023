import { NT4_Client } from "./NT4.js";

const nodeRobotToDashboardTopic = "/nodeselector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/nodeselector/node_dashboard_to_robot";
const substationSelectionRobotToDashboard =
  "/nodeselector/substation_selection_robot_to_dashboard";
const substationSelectionDashboardToRobot =
  "/nodeselector/substation_selection_dashboard_to_robot";

let active = null;
let tipped = null;
let hasRemoved = false;

function displayActive(index) {
  active = index;
  Array.from(document.getElementsByClassName("active")).forEach((element) => {
    element.classList.remove("active");
  });
  if (index !== null) {
    document.getElementsByTagName("td")[index].classList.add("active");
//    displayActiveSubstation(null)
  } else {
    sendActive(-1)
  }

}

function sendActive(index) {
  if (index !== active) {
    client.addSample(nodeDashboardToRobotTopic, index);
  }
}

function displayActiveSubstation(index) {
    tipped = index
    Array.from(document.getElementsByClassName("substation")).forEach((element) => {
        element.classList.remove("tipped");
      });

    if (index !== null) {
        document.getElementsByClassName("substation")[index].classList.add("tipped");
//        displayActive(null)
    } else {
        sendSelectedSubstation(-1)
    }
}

function sendSelectedSubstation(index) {
  if (index !== tipped){
    client.addSample(substationSelectionDashboardToRobot, index);
  }

}

let client = new NT4_Client(
  window.location.hostname,
  "NodeSelector",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === nodeRobotToDashboardTopic) {
      document.body.style.backgroundColor = "white";
      if (value == -1){
        value = null
      }
      displayActive(value);
    } else if (topic.name == substationSelectionRobotToDashboard) {
    if (value == -1){
            value = null
          }
      displayActiveSubstation(value);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    displayActive(null);
    displayActiveSubstation(null);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [nodeRobotToDashboardTopic, substationSelectionRobotToDashboard],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int");
  client.publishTopic(substationSelectionDashboardToRobot, "int");
  client.connect();

  // Add node click listeners
  Array.from(document.getElementsByClassName("node")).forEach((cell, index) => {
    cell.addEventListener("click", () => {
      sendActive(index);
    });
    cell.addEventListener("contextmenu", (event) => {
      event.preventDefault();
      sendActive(index);
    });
  });

  // Add node touch listeners
  [("touchstart", "touchmove")].forEach((eventString) => {
    document.body.addEventListener(eventString, (event) => {
      if (event.touches.length > 0) {
        let x = event.touches[0].clientX;
        let y = event.touches[0].clientY;
        Array.from(document.getElementsByClassName("node")).forEach(
          (cell, index) => {
            let rect = cell.getBoundingClientRect();
            if (
              x >= rect.left &&
              x <= rect.right &&
              y >= rect.top &&
              y <= rect.bottom
            ) {
              sendActive(index);
            }
          }
        );
      }
    });
  });

  // Add cone orientation listeners
//  const coneOrientationDiv =
//    document.getElementsByClassName("cone-orientation")[0];
//  coneOrientationDiv.addEventListener("click", () => {
//    sendSelectedSubstation(coneOrientationDiv.index());
//  });
//  coneOrientationDiv.addEventListener("contextmenu", (event) => {
//    event.preventDefault();
//    sendSelectedSubstation(coneOrientationDiv.index());
//  });
//  coneOrientationDiv.addEventListener("touchstart", () => {
//    event.preventDefault();
//    sendSelectedSubstation(coneOrientationDiv.index());
//  });

  Array.from(document.getElementsByClassName("substation")).forEach((cell, index) => {
      cell.addEventListener("click", () => {
        sendSelectedSubstation(index);
      });
      cell.addEventListener("contextmenu", (event) => {
        event.preventDefault();
        sendSelectedSubstation(index);
      });
    });

    // Add node touch listeners
    [("touchstart", "touchmove")].forEach((eventString) => {
      document.body.addEventListener(eventString, (event) => {
        if (event.touches.length > 0) {
          let x = event.touches[0].clientX;
          let y = event.touches[0].clientY;
          Array.from(document.getElementsByClassName("substation")).forEach(
            (cell, index) => {
              let rect = cell.getBoundingClientRect();
              if (
                x >= rect.left &&
                x <= rect.right &&
                y >= rect.top &&
                y <= rect.bottom
              ) {
                sendSelectedSubstation(index);
              }
            }
          );
        }
      });
    });
});
