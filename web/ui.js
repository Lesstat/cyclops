var map = L.map("mapid", { closePopupOnClick: false }).setView(
  [48.7456643, 9.1070856],
  13
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
  listOfRoutes = [];
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
        Math.round(xmlhttp.response.length / 1000) / 10;
      document.getElementById("route_height").innerHTML =
        xmlhttp.response.height / 10;
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
  listOfRoutes = [];
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
      let coord = configToCoords(values.map(val => val / 100));

      drawDot(coord.x, coord.y, "blue");

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
      coord = configToCoords(values.map(val => val / 100));

      drawDot(coord.x, coord.y, "green");

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
  xmlhttp.open("GET", "/alternative/" + kind + "?s=" + s + "&t=" + t);
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
    "#006837"
  ];
  return colors[number % colors.length];
}

var listOfRoutes = [];

function triangleSplitting() {
  geoJson.clearLayers();
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      listOfRoutes = [];
      let filter = document.getElementById("filter").checked;
      drawTriangle();

      let canvas = document.getElementById("triangleSelector");
      let ctx = canvas.getContext("2d");

      let routes = xmlhttp.response;
      for (var rc in routes) {
        let col = rainbow(rc);

        let myStyle = {
          color: col,
          weight: 4,
          opacity: 1
        };

        let values = routes[rc].config.split("/");
        let coord = configToCoords(values);

        drawDot(coord.x, coord.y, col);

        for (let par in routes[rc].parents) {
          let parConfig = routes[rc].parents[par].split("/");
          let parCoord = configToCoords(parConfig);
          ctx.beginPath();
          ctx.moveTo(coord.x, coord.y);
          ctx.lineTo(parCoord.x, parCoord.y);
          ctx.stroke();
        }

        if (!filter || (filter && routes[rc].selected)) {
          let geoRoute = L.geoJSON(routes[rc].route.route.geometry, {
            style: myStyle
          });
          geoJson.addLayer(geoRoute);
          listOfRoutes.push({
            point: coord,
            route: geoRoute,
            config: values,
            cost: {
              length: routes[rc].route.length,
              height: routes[rc].route.height,
              unsuitability: routes[rc].route.unsuitability
            }
          });
        }
      }

      document.getElementById("shared").innerHTML = "Unknown";
      document.getElementById("frechet").innerHTML = "Unknown";
    } else {
      document.getElementById("route_length").innerHTML = "Unknown";
      document.getElementById("route_height").innerHTML = "Unknown";
      document.getElementById("route_unsuitability").innerHTML = "Unknown";
    }
  };
  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  let threshold = document.getElementById("sharingThreshold").value;
  let maxSplits = document.getElementById("maxSplits").value;
  xmlhttp.open(
    "GET",
    "/splitting" +
      "?s=" +
      s +
      "&t=" +
      t +
      "&threshold=" +
      threshold +
      "&maxSplits=" +
      maxSplits
  );
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
const lengthCorner = { x: 5, y: 505 };
const heightCorner = { x: 505, y: 505 };
const unsuitabilityCorner = { x: 252, y: 22 };
const center = configToCoords([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0]);

function drawTriangle() {
  let canvas = document.getElementById("triangleSelector");
  let ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

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
  let lengthSpan = document.getElementById("length_percent");
  let heightSpan = document.getElementById("height_percent");
  let unsuitabilitySpan = document.getElementById("road_percent");

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
      let conf = bestRoute.config;
      lengthSpan.innerHTML = conf[0] * 100;
      heightSpan.innerHTML = conf[1] * 100;
      unsuitabilitySpan.innerHTML = conf[2] * 100;

      let cost = bestRoute.cost;

      document.getElementById("route_length").innerHTML =
        Math.round(cost.length / 1000) / 10;
      document.getElementById("route_height").innerHTML = cost.height / 10;
      document.getElementById("route_unsuitability").innerHTML =
        cost.unsuitability;
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
      unsuitabilityCorner.y * values[2]
  };
}
function scalingTriangulation() {
  geoJson.clearLayers();
  let xmlhttp = new XMLHttpRequest();

  xmlhttp.responseType = "json";
  xmlhttp.onload = function() {
    if (xmlhttp.status == 200) {
      listOfRoutes = [];
      drawTriangle();

      let canvas = document.getElementById("triangleSelector");
      let ctx = canvas.getContext("2d");

      let points = xmlhttp.response.points;
      let triangles = xmlhttp.response.triangles;

      for (let t in triangles) {
        ctx.fillStyle = "black";
        let point1 = configToCoords(
          points[triangles[t].point1].conf.split("/")
        );
        let point2 = configToCoords(
          points[triangles[t].point2].conf.split("/")
        );
        let point3 = configToCoords(
          points[triangles[t].point3].conf.split("/")
        );
        ctx.beginPath();
        ctx.moveTo(point1.x, point1.y);
        ctx.lineTo(point2.x, point2.y);
        ctx.lineTo(point3.x, point3.y);
        if (triangles[t].filled) {
          ctx.fillStyle = rainbow(triangles[t].point1);
          ctx.fill();
        } else {
          ctx.stroke();
        }
      }
      for (let p in points) {
        let col = rainbow(p);

        let myStyle = {
          color: col,
          weight: 4,
          opacity: 1
        };

        let values = points[p].conf.split("/");
        let coord = configToCoords(values);
        drawDot(coord.x, coord.y, col);

        let geoRoute = L.geoJSON(points[p].route.route.geometry, {
          style: myStyle
        });

        geoJson.addLayer(geoRoute);
        listOfRoutes.push({
          point: coord,
          route: geoRoute,
          config: values,
          cost: {
            length: points[p].route.length,
            height: points[p].route.height,
            unsuitability: points[p].route.unsuitability
          }
        });
      }
    }
  };

  let s = document.getElementById("start").innerHTML;
  let t = document.getElementById("end").innerHTML;
  let maxSplits = document.getElementById("maxSplits").value;
  let maxLevel = document.getElementById("maxLevel").value;
  let uri = "/scaled" + "?s=" + s + "&t=" + t + "&maxSplits=" + maxSplits;

  if (maxLevel > 0) {
    uri += "&maxLevel=" + maxLevel;
  }

  xmlhttp.open("GET", uri);
  xmlhttp.send();
}
