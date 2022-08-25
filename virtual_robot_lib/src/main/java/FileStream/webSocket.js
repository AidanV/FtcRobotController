//let socket = new WebSocket("ws://localhost:8080");
//
//socket.onopen = function(e) {
//  alert("[open] Connection established");
//  alert("Sending to server");
//  socket.send("My name is John");
//};
//
//socket.onmessage = function(event) {
////  alert(`[message] Data received from server: ${event.data}`);
////  var textBox = document.getElementById("textBox");
//  document.getElementById('mytextarea').innerHTML = /* await */event.data.text();
////  textBox.text = event.data;
//};
//
//socket.onclose = function(event) {
//  if (event.wasClean) {
//    alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
//  } else {
//    // e.g. server process killed or network down
//    // event.code is usually 1006 in this case
//    alert('[close] Connection died');
//  }
//};
//
//socket.onerror = function(error) {
//  alert(`[error] ${error.message}`);
//};

var socket;
    if (window.WebSocket) {
        socket = new WebSocket("ws://localhost:8080/myapp");
        socket.onmessage = async function (event) {
<!--            alert("Received data from websocket: " + event.data);-->
            document.getElementById("thing").innerText = await event.data.text();
        }
        socket.onopen = function (event) {
            alert("Web Socket opened!");
        };
        socket.onclose = function (event) {
            alert("Web Socket closed.");
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
