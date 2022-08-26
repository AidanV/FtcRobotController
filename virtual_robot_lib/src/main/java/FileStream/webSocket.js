var webSocketSetup = (handleData) => {
  var socket;
  if (window.WebSocket) {
      socket = new WebSocket("ws://localhost:8080/index.html");
      socket.onmessage = async function (event) {
//            alert("Received data from websocket: "  /*+ event.data*/);
//            document.getElementById("thing").innerText = await event.data.text();

//            position = 0.5;//

//            console.log(JSON.parse(event.data.text())["data"]["power"]);
//            console.log(Buffer.from(event.data))

          event.data.text().then((val) => {
            handleData(JSON.parse(val))
          })

//            scene = createScene();
      }
      socket.onopen = function (event) {
//            alert("Web Socket opened!");
      };
      socket.onclose = function (event) {
//            alert("Web Socket closed.");
      };
  } else {
      alert("Your browser does not support Websockets. (Use Chrome)");
  }

  function send(message) {
      if (!window.WebSocket) {
          return;
      }
      if (socket.readyState == WebSocket.OPEN) {
          socket.send(message);
      } else {
          alert("The socket is not open.");
      }
  }
}
