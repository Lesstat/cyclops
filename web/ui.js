var map = L.map("mapid", { closePopupOnClick: false }).setView(
  [48.7456643, 9.1070856],
  15
);
var start = true;

var startPopup = L.popup({ autoClose: false });
var endPopup = L.popup({ autoClose: false });
var geoJson = L.layerGroup([]).addTo(map);
var towerLayer = L.layerGroup([]).addTo(map);

L.tileLayer("http://{s}.tile.openstreetmap.org/{id}/{z}/{x}/{y}.png", {
  maxZoom: 18,
  attribution:
    'Map data &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors, ' +
    '<a href="http://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ',
  id: ""
}).addTo(map);

$("input").change(function() {
  calcDistWithCurrentSelection();
});

// panOutMap();

function calcDistWithCurrentSelection() {
  let length = document.querySelector('input[name="length"]').value;
  let height = document.querySelector('input[name="height"]').value;
  let unsuitability = document.querySelector('input[name="unsuitability"]')
    .value;
  geoJson.clearLayers();
  calcDist(length, height, unsuitability);
}

function onMapClick(e) {
  let id;
  id = "start";
  startPopup
    .setLatLng(e.latlng)
    .setContent("Start at " + e.latlng.toString())
    .addTo(map);
  getNode(id, e.latlng);
}

function onRightClick(e) {
  let id = "end";
  endPopup
    .setLatLng(e.latlng)
    .setContent("End at " + e.latlng.toString())
    .addTo(map);
  getNode(id, e.latlng);
}

map.on("click", onMapClick);
map.on("contextmenu", onRightClick);

function getNode(id, latlng) {
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      document.getElementById(id).innerHTML = xmlhttp.responseText;
      calcDistWithCurrentSelection();
    }
  };
  xmlhttp.open(
    "GET",
    "/node_at?lat=" + latlng.lat + "&lng=" + latlng.lng,
    true
  );
  xmlhttp.send();
}

function calcDist(length, height, unsuitability) {
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      let myStyle = {
        color: "#3333FF",
        weight: 5,
        opacity: 0.65
      };
      document.getElementById("route_length").innerHTML =
        xmlhttp.response.length;
      document.getElementById("route_height").innerHTML =
        xmlhttp.response.height;
      document.getElementById("route_unsuitability").innerHTML =
        xmlhttp.response.unsuitability;
      geoJson.addLayer(
        L.geoJSON(xmlhttp.response.route.geometry, { style: myStyle })
      );
    } else {
      document.getElementById("route_length").innerHTML = "Unknown";
      document.getElementById("route_height").innerHTML = "Unknown";
      document.getElementById("route_unsuitability").innerHTML = "Unknown";
    }
  };
  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  xmlhttp.open(
    "GET",
    "/route?s=" +
      s +
      "&t=" +
      t +
      "&length=" +
      length +
      "&height=" +
      height +
      "&unsuitability=" +
      unsuitability,
    true
  );
  xmlhttp.send();
}

function panOutMap() {
  let xmlhttp = new XMLHttpRequest();
  xmlhttp.responseType = "json";
  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      map.fitBounds(xmlhttp.response);
    }
  };

  xmlhttp.open("GET", "/map_coords");
  xmlhttp.send();
}

function randomSample() {
  geoJson.clearLayers();
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      let myStyle1 = {
        color: "#3333FF",
        weight: 5,
        opacity: 0.65
      };
      document.getElementById("blue_conf").innerHTML = xmlhttp.response.config1;
      geoJson.addLayer(
        L.geoJSON(xmlhttp.response.route1.route.geometry, { style: myStyle1 })
      );

      let myStyle2 = {
        color: "#33FF00",
        weight: 5,
        opacity: 0.65
      };
      document.getElementById("green_conf").innerHTML =
        xmlhttp.response.config2;
      geoJson.addLayer(
        L.geoJSON(xmlhttp.response.route2.route.geometry, { style: myStyle2 })
      );
      document.getElementById("shared").innerHTML = xmlhttp.response.shared;
    } else {
      document.getElementById("route_length").innerHTML = "Unknown";
      document.getElementById("route_height").innerHTML = "Unknown";
      document.getElementById("route_unsuitability").innerHTML = "Unknown";
    }
  };
  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  xmlhttp.open("GET", "/alternative/random?s=" + s + "&t=" + t);
  xmlhttp.send();
}

function initializeCanvas() {
  let canvas = document.getElementById("triangleSelector");
  canvas.addEventListener("mousemove", moveDot);
  canvas.addEventListener("mouseup", mouseUp);
  canvas.addEventListener("mousedown", mouseDown);

  drawTriangle();
  drawDot(center.x, center.y);
}
const lengthCorner = { x: 0, y: 200 };
const heightCorner = { x: 200, y: 200 };
const unsuitabilityCorner = { x: 100, y: 27 };
const center = { x: 100, y: 142 };

function drawTriangle() {
  let canvas = document.getElementById("triangleSelector");
  let ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, 200, 200);

  ctx.fillStyle = "lightgrey";
  ctx.beginPath();
  ctx.moveTo(lengthCorner.x, lengthCorner.y);
  ctx.lineTo(heightCorner.x, heightCorner.y);
  ctx.lineTo(unsuitabilityCorner.x, unsuitabilityCorner.y);
  ctx.fill();
}

function drawDot(x, y) {
  let canvas = document.getElementById("triangleSelector");
  let ctx = canvas.getContext("2d");

  ctx.fillStyle = "black";
  ctx.beginPath();
  ctx.arc(x, y, 3, 0, 2 * Math.PI);
  ctx.fill();
}

var clicked = false;

function mouseDown(event) {
  clicked = true;
  moveDot(event);
}

function mouseUp(event) {
  clicked = false;
}

function moveDot(event) {
  if (clicked && event) {
    drawTriangle();
    drawDot(event.offsetX, event.offsetY);
  }
}
