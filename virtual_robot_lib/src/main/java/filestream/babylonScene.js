var canvas = document.getElementById("renderCanvas");



var sceneObjects = {};

var createScene = function () {
    var scene = new BABYLON.Scene(engine);
//    JSONObject data = JSON.parse({
//                                 "employees":[
//                                   {"firstName":"John", "lastName":"Doe"},
//                                   {"firstName":"Anna", "lastName":"Smith"},
//                                   {"firstName":"Peter", "lastName":"Jones"}
//                                 ]
//                                 });

    var camera = new BABYLON.FreeCamera("camera1", new BABYLON.Vector3(0, 5, -10), scene);
		camera.setTarget(BABYLON.Vector3.Zero());
		camera.attachControl(canvas, true);
    var light = new BABYLON.HemisphericLight("light1", new BABYLON.Vector3(0, 1, 0), scene);
		light.intensity = 0.7;
    var ground = BABYLON.Mesh.CreateGround("ground1", 6, 6, 2, scene);

    sceneObjects.box = BABYLON.Mesh.CreateBox("box", 4.0, scene);



	var url;
	var fileName;


//-- BoomBox.gltf
	url = "https://raw.githubusercontent.com/BabylonJS/Babylon.js/master/Playground/scenes/BoomBox/";
	fileName = "BoomBox.gltf";

	BABYLON.SceneLoader.ImportMesh("", url, fileName, scene, function (newMeshes) {
		var mesh = newMeshes[0];
			mesh.position.copyFromFloats(0, 1, 0);
			mesh.scaling.copyFromFloats(100,100,100);

		camera.target = mesh;
		console.log(mesh)
	});
    return scene;
};

var engine = new BABYLON.Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true });
var scene = createScene();

engine.runRenderLoop(function () {
	if (scene) {
		scene.render();

	}
});

// Resize
window.addEventListener("resize", function () {
	engine.resize();
});


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
              var position = parseFloat(JSON.parse(val)["data"]["power"]);
              sceneObjects.box.position = new BABYLON.Vector3(position, 2, 2);
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

