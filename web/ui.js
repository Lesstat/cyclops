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

// panOutMap();

function calcDistWithCurrentSelection() {
  let length = document.getElementById("length_percent").innerHTML;
  let height = document.getElementById("height_percent").innerHTML;
  let unsuitability = document.getElementById("road_percent").innerHTML;
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
        Math.round(xmlhttp.response.length / 100) / 10;
      document.getElementById("route_height").innerHTML =
        xmlhttp.response.height;
      document.getElementById("route_unsuitability").innerHTML =
        xmlhttp.response.unsuitability;
      geoJson.clearLayers();
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

function alternativeRoutes(kind) {
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
      drawTriangle();
      document.getElementById("blue_conf").innerHTML = xmlhttp.response.config1;
      var values = xmlhttp.response.config1.split("/");
      var x =
        lengthCorner.x * values[0] / 100 +
        heightCorner.x * values[1] / 100 +
        unsuitabilityCorner.x * values[2] / 100;
      var y =
        lengthCorner.y * values[0] / 100 +
        heightCorner.y * values[1] / 100 +
        unsuitabilityCorner.y * values[2] / 100;

      drawDot(x, y, "blue");

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

      values = xmlhttp.response.config2.split("/");
      x =
        lengthCorner.x * values[0] / 100 +
        heightCorner.x * values[1] / 100 +
        unsuitabilityCorner.x * values[2] / 100;
      y =
        lengthCorner.y * values[0] / 100 +
        heightCorner.y * values[1] / 100 +
        unsuitabilityCorner.y * values[2] / 100;

      drawDot(x, y, "green");

      geoJson.addLayer(
        L.geoJSON(xmlhttp.response.route2.route.geometry, { style: myStyle2 })
      );
      document.getElementById("shared").innerHTML = xmlhttp.response.shared;
      document.getElementById("frechet").innerHTML = xmlhttp.response.frechet;
    } else {
      document.getElementById("route_length").innerHTML = "Unknown";
      document.getElementById("route_height").innerHTML = "Unknown";
      document.getElementById("route_unsuitability").innerHTML = "Unknown";
    }
  };
  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  xmlhttp.open("GET", "/alternative/" + kind + "?s=" + s + "&t=" + t);
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
const lengthCorner = { x: 5, y: 195 };
const heightCorner = { x: 205, y: 195 };
const unsuitabilityCorner = { x: 105, y: 22 };
const center = { x: 100, y: 142 };

function drawTriangle() {
  let canvas = document.getElementById("triangleSelector");
  let ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, 210, 210);

  ctx.fillStyle = "lightgrey";
  ctx.beginPath();
  ctx.moveTo(lengthCorner.x, lengthCorner.y);
  ctx.lineTo(heightCorner.x, heightCorner.y);
  ctx.lineTo(unsuitabilityCorner.x, unsuitabilityCorner.y);
  ctx.fill();
}

function drawDot(x, y, style) {
  if (style === undefined) {
    style = "black";
  }
  let canvas = document.getElementById("triangleSelector");
  let ctx = canvas.getContext("2d");

  ctx.fillStyle = style;
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
  calcDistWithCurrentSelection();
}

function moveDot(event) {
  if (clicked && event) {
    drawTriangle();
    drawDot(event.offsetX, event.offsetY);
    let point = { x: event.offsetX, y: event.offsetY };
    let lengthArea = triangleArea(point, heightCorner, unsuitabilityCorner);
    let heightArea = triangleArea(point, lengthCorner, unsuitabilityCorner);
    let unsuitabilityArea = triangleArea(point, lengthCorner, heightCorner);

    let areaSum = lengthArea + heightArea + unsuitabilityArea;

    let lengthPercent = Math.round(lengthArea / areaSum * 100);
    let heightPercent = Math.round(heightArea / areaSum * 100);

    let unsuitabilityPercent = Math.round(unsuitabilityArea / areaSum * 100);

    let lengthSpan = document.getElementById("length_percent");
    let heightSpan = document.getElementById("height_percent");
    let unsuitabilitySpan = document.getElementById("road_percent");

    lengthSpan.innerHTML = lengthPercent;
    heightSpan.innerHTML = heightPercent;
    unsuitabilitySpan.innerHTML = unsuitabilityPercent;
  }
}

function triangleArea(p1, p2, p3) {
  let dist = function(p1, p2) {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
  };

  let a = dist(p1, p2);
  let b = dist(p1, p3);
  let c = dist(p2, p3);

  let s = (a + b + c) / 2;

  return Math.sqrt(s * (s - a) * (s - b) * (s - c));
}
