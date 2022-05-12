var map = L.map("mapid", { closePopupOnClick: false }).setView(
  [48.7456643, 9.1070856],
  13
);
var start = true;

var startMarker = L.marker();
var endMarker = L.marker();
var geoJson = L.layerGroup([]).addTo(map);

let canvasRgb = document.getElementById("triangleSelectorRGB");

L.tileLayer("https://b.tile.openstreetmap.de/{z}/{x}/{y}.png", {
  maxZoom: 18,
  attribution:
    'Map data &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors, ' +
    '<a href="http://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ',
  id: "",
}).addTo(map);

document.addEventListener("keydown", (event) => {
  if (event.ctrlKey && event.altKey && event.key == "d") {
    let debugLog = document.getElementById("debuglog");
    if (debugLog.hasAttribute("hidden")) {
      debugLog.removeAttribute("hidden");
    } else {
      debugLog.setAttribute("hidden", "");
    }
  }
});

function calcDistWithCurrentSelection() {
  let length = document.getElementById("length_percent").innerHTML;
  let height = document.getElementById("height_percent").innerHTML;
  let unsuitability = document.getElementById("road_percent").innerHTML;
  calcDist(length, height, unsuitability);
}

function onMapClick(e) {
  let id;
  id = "start";
  startMarker
    .setLatLng(e.latlng)
    .bindPopup("Start at " + e.latlng.toString())
    .addTo(map);
  getNode(id, e.latlng);
}

function onRightClick(e) {
  let id = "end";
  endMarker
    .setLatLng(e.latlng)
    .bindPopup("End at " + e.latlng.toString())
    .addTo(map);
  getNode(id, e.latlng);
}

map.on("click", onMapClick);
map.on("contextmenu", onRightClick);

function getNode(id, latlng) {
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.onload = function () {
    if (xmlhttp.status == 200) {
      addToDebugLog("getNode", xmlhttp.response.debug);
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
  listOfRoutes = [];
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function () {
    if (xmlhttp.status == 200) {
      addToDebugLog("single route", xmlhttp.response.debug);
      let myStyle = {
        color: "#3333FF",
        weight: 5,
        opacity: 0.65,
      };
      document.getElementById("route_length").innerHTML =
        Math.round(xmlhttp.response.costs[0] / 100) / 10;
      document.getElementById("route_height").innerHTML =
        Math.round(xmlhttp.response.costs[1] * 100) / 100;
      document.getElementById("route_unsuitability").innerHTML =
        Math.round(xmlhttp.response.costs[2] * 100) / 100;
      clearOverlays();
      geoJson.addLayer(
        L.geoJSON(xmlhttp.response.route, {
          style: myStyle,
          onEachFeature: createHeightChart,
        })
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
  xmlhttp.onload = function () {
    if (xmlhttp.status == 200) {
      map.fitBounds(xmlhttp.response);
    }
  };

  xmlhttp.open("GET", "/map_coords");
  xmlhttp.send();
}

function rainbow(number) {
  let colors = [
    "#543005",
    "#8c510a",
    "#bf812d",
    "#dfc27d",
    "#f628c3",
    "#000000",
    "#c7ea25",
    "#80cdc1",
    "#35978f",
    "#01665e",
    "#003c30",
    "#a50026",
    "#d73027",
    "#f46d43",
    "#fdae61",
    "#fee08b",
    "#333333",
    "#d9ef8b",
    "#a6d96a",
    "#66bd63",
    "#1a9850",
    "#006837",
  ];
  return colors[number % colors.length];
}

function gradientToColor(config) {
  let rgb = config.map((val) => Math.round(val * 255));
  let myMax = Math.max(...rgb);
  var decColor =
    0x1000000 +
    Math.round((rgb[0] * 255.0) / myMax) +
    0x100 * Math.round((rgb[1] * 255.0) / myMax) +
    0x10000 * Math.round((rgb[2] * 255.0) / myMax);
  return "#" + decColor.toString(16).substr(1);
}

var listOfRoutes = [];

function initializeCanvas() {
  canvasRgb.addEventListener("mousemove", moveDot);
  canvasRgb.addEventListener("mouseup", mouseUp);
  canvasRgb.addEventListener("mousedown", mouseDown);

  drawTriangle(canvasRgb);
  drawDot(canvasRgb, center.x, center.y);
}

const lengthCorner = { x: 20, y: canvasRgb.height - 30 };
const heightCorner = { x: canvasRgb.width - 20, y: canvasRgb.height - 30 };
const unsuitabilityCorner = { x: (lengthCorner.x + heightCorner.x) / 2, y: 50 };
const center = configToCoords([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0]);

function drawTriangle(canvas) {
  let ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "lightgrey";
  ctx.beginPath();
  ctx.moveTo(lengthCorner.x, lengthCorner.y);
  ctx.lineTo(heightCorner.x, heightCorner.y);
  ctx.lineTo(unsuitabilityCorner.x, unsuitabilityCorner.y);
  ctx.fill();
  ctx.font = "20px sans-serif";

  ctx.fillStyle = "blue";
  ctx.fillText("Length", 0, lengthCorner.y + 20);
  ctx.fillStyle = "red";
  ctx.fillText(
    "Unsuitability",
    unsuitabilityCorner.x - 60,
    unsuitabilityCorner.y - 10
  );

  ctx.fillStyle = "green";
  ctx.fillText("Height", heightCorner.x - 50, heightCorner.y + 20);
}

function drawDot(canvas, x, y, style) {
  if (style === undefined) {
    style = "black";
  }
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
  let lengthSpan = document.getElementById("length_percent");
  let heightSpan = document.getElementById("height_percent");
  let unsuitabilitySpan = document.getElementById("road_percent");

  if (clicked && event) {
    drawTriangle(canvasRgb);
    drawDot(canvasRgb, event.offsetX, event.offsetY);
    let point = { x: event.offsetX, y: event.offsetY };
    let lengthArea = triangleArea(point, heightCorner, unsuitabilityCorner);
    let heightArea = triangleArea(point, lengthCorner, unsuitabilityCorner);
    let unsuitabilityArea = triangleArea(point, lengthCorner, heightCorner);

    let areaSum = lengthArea + heightArea + unsuitabilityArea;

    let lengthPercent = Math.round((lengthArea / areaSum) * 100);
    let heightPercent = Math.round((heightArea / areaSum) * 100);

    let unsuitabilityPercent = Math.round((unsuitabilityArea / areaSum) * 100);

    lengthSpan.innerHTML = lengthPercent;
    heightSpan.innerHTML = heightPercent;
    unsuitabilitySpan.innerHTML = unsuitabilityPercent;
  } else if (event && listOfRoutes.length) {
    let point = { x: event.offsetX, y: event.offsetY };
    let minDist = 1000;
    let minIndex;
    for (let index in listOfRoutes) {
      listOfRoutes[index].route.setStyle({ weight: 4 });
      let dis = dist(listOfRoutes[index].point, point);
      if (minDist > dis) {
        minDist = dis;
        minIndex = index;
      }
    }
    if (minDist <= 5) {
      let bestRoute = listOfRoutes[minIndex];
      bestRoute.route.setStyle({ weight: 10 });
      createHeightChart(
        bestRoute.route.toGeoJSON().features[0],
        bestRoute.route
      );
      let conf = bestRoute.config;
      lengthSpan.innerHTML = Math.round(conf[0] * 10000) / 100;
      heightSpan.innerHTML = Math.round(conf[1] * 10000) / 100;
      unsuitabilitySpan.innerHTML = Math.round(conf[2] * 10000) / 100;

      let cost = bestRoute.cost;
      document.getElementById("route_length").innerHTML =
        Math.round(cost[0] / 100) / 10;
      document.getElementById("route_height").innerHTML =
        Math.round(cost[1] * 100) / 100;
      document.getElementById("route_unsuitability").innerHTML =
        Math.round(cost[2] * 100) / 100;
    }
  }
}

function dist(p1, p2) {
  return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
}

function triangleArea(p1, p2, p3) {
  let a = dist(p1, p2);
  let b = dist(p1, p3);
  let c = dist(p2, p3);

  let s = (a + b + c) / 2;

  return Math.sqrt(s * (s - a) * (s - b) * (s - c));
}

function configToCoords(values) {
  return {
    x:
      lengthCorner.x * values[0] +
      heightCorner.x * values[1] +
      unsuitabilityCorner.x * values[2],
    y:
      lengthCorner.y * values[0] +
      heightCorner.y * values[1] +
      unsuitabilityCorner.y * values[2],
  };
}

function addToDebugLog(requestType, message) {
  if (!message) {
    return;
  }
  let debugLog = document.getElementById("debuglog");
  let content = "Debug log for " + requestType + " request\n";
  content += message;
  content += "\nEnd of log for " + requestType + " request\n";
  content += "=============================================\n";
  debugLog.value += content;
}

function drawTriangles(ctx, points, triangles, gradient) {
  for (let t in triangles) {
    ctx.fillStyle = "black";
    let config1 = points[triangles[t].point1].conf;
    let config2 = points[triangles[t].point2].conf;
    let config3 = points[triangles[t].point3].conf;
    let point1 = configToCoords(config1);
    let point2 = configToCoords(config2);
    let point3 = configToCoords(config3);

    ctx.beginPath();
    ctx.moveTo(point1.x, point1.y);
    ctx.lineTo(point2.x, point2.y);
    ctx.lineTo(point3.x, point3.y);
    let filled = false;
    if (gradient) {
      if (triangles[t].noChildren) {
        let center = [];
        for (let i = 0; i < 3; i++) {
          center.push((config1[i] + config2[i] + config3[i]) / 3);
        }
        ctx.fillStyle = gradientToColor(center);
        filled = true;
      }
    } else {
      if (triangles[t].noMoreRoutes) {
        ctx.fillStyle = rainbow(triangles[t].point1);
        filled = true;
      }
    }
    if (filled) {
      ctx.fill();
    } else {
      ctx.stroke();
    }
  }
}
function overlapChange() {
  let slider = document.getElementById("maxOverlap");
  let span = document.getElementById("overlap");
  span.innerHTML = slider.value + "%";
}

let heightValues = [];
let labels = [];
let myLineChart = {};
let heightMarker;
function createHeightChart(json, layer) {
  layer.bindPopup("I have a height chart attached");
  let coords = json.geometry.coordinates;

  if (myLineChart.destroy) myLineChart.destroy();
  heightValues = [];
  labels = [];
  let dist = 0;
  for (let c in coords) {
    if (c > 0) {
      let node = L.latLng(coords[c][1], coords[c][0]);
      let lastNode = L.latLng(coords[c - 1][1], coords[c - 1][0]);
      dist += lastNode.distanceTo(node);
    }
    labels.push(Math.round(dist * 10) / 10);
    heightValues.push(coords[c][2]);
  }

  let ctx = document.getElementById("heightChart").getContext("2d");
  myLineChart = new Chart(ctx, {
    type: "line",
    data: {
      labels: labels,
      datasets: [
        {
          label: "Height Profile",
          borderColor: "rgb(255, 99, 132)",
          pointBackgroundColor: "rgb(0,0,0)",
          data: heightValues,
          lineTension: 0,
          pointRadius: 3,
        },
      ],
    },
    options: {
      onHover: function (event, points) {
        if (points.length > 0) {
          let index = points[0]._index;

          let ll = L.latLng(coords[index][1], coords[index][0]);
          if (heightMarker) {
            heightMarker.setLatLng(ll);
          } else {
            heightMarker = L.marker(ll).addTo(map);
          }
        }
      },
    },
  });
  myLineChart.update();
}
function clearOverlays() {
  geoJson.clearLayers();
  if (heightMarker) heightMarker.remove();
  heightMarker = undefined;
  if (myLineChart.destroy) myLineChart.destroy();
}

function enumerateRoutes() {
  clearOverlays();
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function () {
    if (xmlhttp.status == 200) {
      addToDebugLog("enumeration", xmlhttp.response.debug);
      listOfRoutes = [];
      drawTriangle(canvasRgb);

      let points = xmlhttp.response.points;
      let coords = [];

      for (let p in points) {
        if (!points[p].selected) {
          continue;
        }
        let values = points[p].conf;
        let col = gradientToColor(values);
        let coord = configToCoords(values);
        coords.push(coord);
        drawDot(canvasRgb, coord.x, coord.y, col);

        let myStyle = {
          color: col,
          weight: 4,
          opacity: 0.7,
        };
        let geoRoute = L.geoJSON(points[p].route.route.geometry, {
          style: myStyle,
        });
        geoJson.addLayer(geoRoute);
        listOfRoutes.push({
          point: coord,
          route: geoRoute,
          config: values,
          cost: points[p].route.costs,
        });
      }
    }
  };

  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  let maxRefinements = document.getElementById("maxRefinements").value;
  let maxOverlap = document.getElementById("maxOverlap").value;
  let important = document.getElementById("important").value;

  let uri =
    "/enumerate" +
    "?s=" +
    s +
    "&t=" +
    t +
    "&maxRoutes=" +
    maxRefinements +
    "&maxOverlap=" +
    maxOverlap +
    "&important=" +
    important;

  xmlhttp.open("GET", uri);
  xmlhttp.send();
}

function zoomOutToFullGraph() {
  console.log("zoom out");
  let xmlhttp = new XMLHttpRequest();
  xmlhttp.responseType = "json";
  xmlhttp.onload = function () {
    if (xmlhttp.status == 200) {
      let bbox = xmlhttp.response;
      map.fitBounds([
        [bbox.lat_min, bbox.lng_min],
        [bbox.lat_max, bbox.lng_max],
      ]);
    }
  };
  xmlhttp.open("GET", "/graph_coords");
  xmlhttp.send();
}

overlapChange();
zoomOutToFullGraph();
