import { NT4_Client } from "./NT4.js";

const nodeRobotToDashboardTopic = "/nodeselector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/nodeselector/node_dashboard_to_robot";
const coneTippedRobotToDashboardTopic =
  "/nodeselector/cone_tipped_robot_to_dashboard";
const coneTippedDashboardToRobotTopic =
  "/nodeselector/cone_tipped_dashboard_to_robot";

let active = null;
let tipped = null;
let hasRemoved = false;

function displayActive(index) {
  active = index;
  Array.from(document.getElementsByClassName("active")).forEach((element) => {
    element.classList.remove("active");
  });
  if (index !== null) {
    console.log(index)
    document.getElementsByTagName("td")[index].classList.add("active");
    displayTipped(null)
  } else {
    sendActive(-1)
  }

}

function sendActive(index) {
  if (index !== active) {
    client.addSample(nodeDashboardToRobotTopic, index);
  }
}

function displayTipped(index) {
    tipped = index
    Array.from(document.getElementsByClassName("substation")).forEach((element) => {
        element.classList.remove("tipped");
      });

    if (index !== null) {
        document.getElementsByClassName("substation")[index].classList.add("tipped");
        displayActive(null)
    } else {
        toggleTipped(-1)
    }
}

function toggleTipped(index) {
  if (index !== tipped){
    client.addSample(coneTippedDashboardToRobotTopic, index);
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
    } else if (topic.name == coneTippedRobotToDashboardTopic) {
    if (value == -1){
            value = null
          }
      displayTipped(value);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    displayActive(null);
    displayTipped(null);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [nodeRobotToDashboardTopic, coneTippedRobotToDashboardTopic],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int");
  client.publishTopic(coneTippedDashboardToRobotTopic, "int");
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
//    toggleTipped(coneOrientationDiv.index());
//  });
//  coneOrientationDiv.addEventListener("contextmenu", (event) => {
//    event.preventDefault();
//    toggleTipped(coneOrientationDiv.index());
//  });
//  coneOrientationDiv.addEventListener("touchstart", () => {
//    event.preventDefault();
//    toggleTipped(coneOrientationDiv.index());
//  });

  Array.from(document.getElementsByClassName("substation")).forEach((cell, index) => {
      cell.addEventListener("click", () => {
        toggleTipped(index);
      });
      cell.addEventListener("contextmenu", (event) => {
        event.preventDefault();
        toggleTipped(index);
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
                toggleTipped(index);
              }
            }
          );
        }
      });
    });
});
