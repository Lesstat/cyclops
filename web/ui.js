var map = L.map('mapid', { closePopupOnClick: false}).setView([48.7456643, 9.1070856], 15);
var start = true;

var startPopup = L.popup({ autoClose: false });
var endPopup = L.popup({ autoClose: false });
var geoJson = L.layerGroup([]).addTo(map);
var towerLayer = L.layerGroup([]).addTo(map);

L.tileLayer('http://{s}.tile.openstreetmap.org/{id}/{z}/{x}/{y}.png', {
    maxZoom: 18,
    attribution: 'Map data &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors, ' +
	'<a href="http://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ',
    id: ''
}).addTo(map);

$("input").change(function() {
    calcDistWithCurrentSelection();
});

// panOutMap();

function calcDistWithCurrentSelection(){
    var length = document.querySelector('input[name="length"]').value;
    var height = document.querySelector('input[name="height"]').value;
    var unsuitability = document.querySelector('input[name="unsuitability"]').value;
    geoJson.clearLayers(); 
    calcDist(length, height, unsuitability);
}

function onMapClick(e) {
    var id;
    id = "start";
    startPopup.setLatLng(e.latlng).setContent("Start at " + e.latlng.toString()).addTo(map);
    getNode(id, e.latlng) ;
}

function onRightClick(e){
    var id = "end";
    endPopup.setLatLng(e.latlng).setContent("End at " + e.latlng.toString()).addTo(map);
    getNode(id, e.latlng) ;
}


map.on('click', onMapClick);
map.on('contextmenu', onRightClick);

function getNode(id, latlng){
    
    var xmlhttp = new XMLHttpRequest();
    
    xmlhttp.onload = function() {
	if (xmlhttp.status == 200) {
	    document.getElementById(id).innerHTML = xmlhttp.responseText;
	    calcDistWithCurrentSelection();
	}
    };
    xmlhttp.open("GET", "/node_at?lat="+ latlng.lat  + "&lng=" + latlng.lng, true);
    xmlhttp.send();
}

function calcDist(length, height, unsuitability){
    
    var xmlhttp = new XMLHttpRequest();
    
    xmlhttp.responseType = 'json';
    xmlhttp.onload = function() {
	if (xmlhttp.status == 200) {
	    var myStyle = {
		"color": "#3333FF",
		"weight": 5,
		"opacity": 0.65
	    };
	    document.getElementById("route_length").innerHTML = xmlhttp.response.length;
	    document.getElementById("route_height").innerHTML = xmlhttp.response.height;
	    document.getElementById("route_unsuitability").innerHTML = xmlhttp.response.unsuitability;
	    geoJson.addLayer(L.geoJSON(xmlhttp.response.route.geometry, { style: myStyle }));
	}
	else {
	    document.getElementById("route_length").innerHTML = "Unknown";
	    document.getElementById("route_height").innerHTML = "Unknown";
	    document.getElementById("route_unsuitability").innerHTML = "Unknown";
	}
    };
    var s = document.getElementById("start").innerHTML;
    var t = document.getElementById("end").innerHTML;
    xmlhttp.open("GET", "/route?s="+ s
		 + "&t=" + t
		 + "&length=" + length
		 + "&height=" + height
		 + "&unsuitability=" + unsuitability, true);
    xmlhttp.send();
}



function panOutMap(){
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.responseType = "json";
    xmlhttp.onload =  function(){
	if (xmlhttp.status == 200){
	    map.fitBounds(xmlhttp.response);
	}
    };

    xmlhttp.open("GET", "/map_coords");
    xmlhttp.send();
    
}

function randomSample(){

    geoJson.clearLayers(); 
    var xmlhttp = new XMLHttpRequest();
    
    xmlhttp.responseType = 'json';
    xmlhttp.onload = function() {
	if (xmlhttp.status == 200) {
	    var myStyle1 = {
		"color": "#3333FF",
		"weight": 5,
		"opacity": 0.65
	    };
	    document.getElementById("blue_conf").innerHTML = xmlhttp.response.config1;
	    geoJson.addLayer(L.geoJSON(xmlhttp.response.route1.route.geometry, { style: myStyle1 }));

	    var myStyle2 = {
		"color": "#33FF00",
		"weight": 5,
		"opacity": 0.65
	    };
	    document.getElementById("green_conf").innerHTML = xmlhttp.response.config2;
	    geoJson.addLayer(L.geoJSON(xmlhttp.response.route2.route.geometry, { style: myStyle2 }));
	    document.getElementById("shared").innerHTML = xmlhttp.response.shared;
	}
	else {
	    document.getElementById("route_length").innerHTML = "Unknown";
	    document.getElementById("route_height").innerHTML = "Unknown";
	    document.getElementById("route_unsuitability").innerHTML = "Unknown";
	}
    };
    var s = document.getElementById("start").innerHTML;
    var t = document.getElementById("end").innerHTML;
    xmlhttp.open("GET", "/alternative/random?s="+ s + "&t=" + t);
    xmlhttp.send();
}

