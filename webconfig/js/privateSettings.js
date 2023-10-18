




var currentVersion = 14
var midiAccess=null;  // the MIDIAccess object.
var on,
    off;
      //outputs = [];
var deviceName;
var volume = 0;
var currentNote;
var sensorValue = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
var buttonRowWrite; //used to indicate which button command row we'll be writing to below.
var jumpFactorWrite; //to indicate which pressure variably is going to be sent or received
var fingeringWrite; //indicates the instrument for which a fingering pattern is being sent.
var version = "Unknown";
var instrument = 0; //currently selected instrument tab
//var customEnabled = 1; //if custom vibrato is enabled for the current fingering pattern
var ping = 0;
var lsb = 0; //used for reassembling pressure bytes from WARBL

var modalA = document.getElementById('open-modal1');
var modalB = document.getElementById('open-modal2');
var modalC = document.getElementById('open-modal3');
var modalD = document.getElementById('open-modal4');
var modalE = document.getElementById('open-modal5');
var modalF = document.getElementById('open-modal6');
var modalG = document.getElementById('open-modal7');
var modalH = document.getElementById('open-modal8');
var modalI = document.getElementById('open-modal9');
var modalJ = document.getElementById('open-modal10');
var modalK = document.getElementById('open-modal11');
var modalL = document.getElementById('open-modal12');
var modalM = document.getElementById('open-modal13');

// When the user clicks anywhere outside of the modal, close it
//$(document).on('click touchstart', function (e) {
window.onclick = function (event) {
  
   if (event.target == modalA) {
        modalclose(1);} 
  if(event.target == modalB) {
        modalclose(2);  } 
   if(event.target == modalC) {
        modalclose(3);   }
   if(event.target == modalD) {
        modalclose(4);  } 
   if(event.target == modalE) {
        modalclose(5);  }     
    if(event.target == modalF) {
        modalclose(6);  }     
   if(event.target == modalG) {
        modalclose(7);  }     
   if(event.target == modalH) {
        modalclose(8);  }    
   if(event.target == modalI) {
        modalclose(9);  }  
   if(event.target == modalJ) {
        modalclose(10);  }  
   if(event.target == modalK) {
        modalclose(11);  } 
   if(event.target == modalL) {
        modalclose(12);  } 
   if(event.target == modalM) {
        modalclose(13);  }        
        event.stopPropagation();}


$(document).ready(function(){


	updateCells(); // set selects and radio buttons to initial values and enabled/disabled states
	//toggleOn();
	updateSliders();
//$(this).find('i').toggleClass('fa-volume-up fa-volume-off');
	
	

$(".volume").click(function(){
		toggleOn();
$(this).find('i').toggleClass('fa-volume-up fa-volume-off')
});
});


window.addEventListener('load', function() {
  // patch up prefixes
  
// window.AudioContext=window.AudioContext||window.webkitAudioContext;


  if (navigator.requestMIDIAccess)
    navigator.requestMIDIAccess( { sysex: true } ).then( onMIDIInit, onMIDIReject );
  else
    alert("Your browser does not support MIDI. Please use Chrome or Opera, or the free WARBL iOS app.")

} );

function connect() {
		ping = 0;
			document.getElementById("status").style.color = "#F78339";
	document.getElementById("version").style.color = "#F78339";
    document.getElementById("status").innerHTML = "WARBL not detected.";
	document.getElementById("version").innerHTML = "Unknown";
	document.getElementById("current").style.visibility = "hidden";
		document.getElementById("status").style.visibility = "visible";
	  if (navigator.requestMIDIAccess)
    navigator.requestMIDIAccess( { sysex: true } ).then( onMIDIInit, onMIDIReject );
  else
    alert("Your browser does not support MIDI. Please use Chrome or Opera, or the free WARBL iOS app.")
}

function onMIDIInit(midi) {
  midiAccess = midi;


  var haveAtLeastOneDevice=false;
  var inputs=midiAccess.inputs.values();
  for ( var input = inputs.next(); input && !input.done; input = inputs.next()) {
	 deviceName = input.value.name;
	 
	   if(deviceName.includes ("WARBL")){document.getElementById("status").innerHTML = "WARBL Connected.";
  document.getElementById("status").style.color = "#f7c839";
  send(102,126); //tell WARBL to enter communications mode
  setPing(); //start checking to make sure WARBL is still connected
  } 
  
   
		else{document.getElementById("status").style.color = "#F78339";
		document.getElementById("version").style.color = "#F78339";
    	document.getElementById("status").innerHTML = "WARBL not detected.";}

	 
    input.value.onmidimessage = MIDIMessageEventHandler;
    haveAtLeastOneDevice = true;
}


  if (!haveAtLeastOneDevice){
	document.getElementById("status").style.color = "#F78339";
	document.getElementById("version").style.color = "#F78339";
    document.getElementById("status").innerHTML = "WARBL not detected.";
}


midi.onstatechange = midiOnStateChange;

}


function midiOnStateChange(event) {

	if(ping == 1){	
	document.getElementById("status").style.color = "#F78339";
	document.getElementById("version").style.color = "#F78339";
    document.getElementById("status").innerHTML = "WARBL not detected.";
	document.getElementById("version").innerHTML = "Unknown";
	document.getElementById("current").style.visibility = "hidden";
		document.getElementById("status").style.visibility = "visible";
	}
	
	}



function onMIDIReject(err) {
  alert("The MIDI system failed to start. Please refresh the page.");
}


function setPing() {
  setTimeout(function(){ ping = 1; }, 3000); //change ping to 1 after 3 seconds, after which we'll show a disconnnect if the MIDI state changes.
}



function MIDIMessageEventHandler(event) {


var e;
var f;
if (event.data[2] == "undefined") {f = " ";}
else {f = event.data[2];}
if((event.data[0] & 0xf0) == 144){e = "On";} //update the MIDI console
if((event.data[0] & 0xf0) == 128){e = "Off";}
if((event.data[0] & 0xf0) == 176){e = "CC";}
if((event.data[0] & 0xf0) == 224){e = "PB";}
if((event.data[0] & 0xf0) == 192){e = "PC";}
if (!(e == "CC" && (event.data[1] > 101 || event.data[1] < 120))) { 
	document.getElementById("console").innerHTML = (e + " " + ((event.data[0] & 0x0f) + 1) + " " + event.data[1] + " " + f);}
	
	console.log("WARBL_Receive: "+event.data[0]+" "+event.data[1]+" "+event.data[2]);
	
  // Mask off the lower nibble (MIDI channel, which we don't care about yet)
  switch (event.data[0] & 0xf0) {
    case 0x90:
      if (event.data[2]!=0) {  // if velocity != 0, this is a note-on message
        noteOn(event.data[1]);
        return;
      }
      // if velocity == 0, fall thru: it's a note-off.
    case 0x80:
      noteOff(event.data[1]);
      return;
	  
 case 0xB0: //incoming CC from WARBL
  
  
if(parseFloat(event.data[0] & 0x0f) == 6) { //if it's channel 7 it's from WARBL 
 	//console.log("WARBL_Receive: "+data0+" "+data1+" "+data2);
  if (event.data[1] == 115){ //hole covered info from WARBL
  	var byteA = event.data[2];
	for (var m=0; m<7; m++) {
		if(bit_test(byteA, m) == 1){document.getElementById("dot" + m).style.backgroundColor = "blue";
		if (m == 0) {document.getElementById("dot0").style.opacity = "0.8";}}	
			else {document.getElementById("dot" + m).style.backgroundColor = "#333";
				if (m == 0) {document.getElementById("dot0").style.opacity = "0";} 
			}}}

    if (event.data[1] == 114){ //hole covered info from WARBL
	for (var n=0; n<2; n++) {
  	var byteB = event.data[2];
		if(bit_test(byteB, n) == 1){document.getElementById("dot" + (7 + n)).style.backgroundColor = "blue";}	
			else{
			if (n==1){document.getElementById("dot" + (7 + n)).style.backgroundColor = "#181818";}
			if (n==0){document.getElementById("dot" + (7 + n)).style.backgroundColor = "#333";}}}}
  
  
  
 else if (event.data[1] == 102){ //parse based on received CC
  		if (event.data[2] > 19 && event.data[2] < 30) {
	  		document.getElementById("v" + (event.data[2]-19)).innerHTML = "MAX";  //set sensor value field to max if message is received from WARBL
	  		checkMax((event.data[2]-19));}
	 
	  	for (var i=0; i<3; i++){ // update the three selected fingering patterns if prompted by the tool.
          	if (event.data[2] == 30 + i) {
          		fingeringWrite = i;
          		}          		
	  			
          if (event.data[2] > 32 && event.data[2] < 60) {
			if (fingeringWrite == i){
				document.getElementById("fingeringSelect" + i).value = event.data[2] - 33;
			}
          updateCells(); } //update any dependant fields	
          }	
        
	  		  		
 
	  if (event.data[2] == 60){
		  document.getElementById("fingering0").checked = true;
	 	 instrument = 0;
		 document.getElementById("instrument0").style.display= "block";
		document.getElementById("instrument1").style.display= "none";
		document.getElementById("instrument2").style.display= "none";
	
		document.getElementById("key0").style.display= "block";
		document.getElementById("key1").style.display= "none";
		document.getElementById("key2").style.display= "none";
		 advancedOkay(); //turn off the advanced tab	
		 updateCells();
		 }
	  if (event.data[2] == 61){
		  document.getElementById("fingering1").checked = true;
		  instrument = 1;
		 document.getElementById("instrument0").style.display= "none";
		document.getElementById("instrument1").style.display= "block";
		document.getElementById("instrument2").style.display= "none";
	
		document.getElementById("key0").style.display= "none";
		document.getElementById("key1").style.display= "block";
		document.getElementById("key2").style.display= "none";		  
		  advancedOkay(); //turn off the advanced tab	
		 updateCells();}
	  if (event.data[2] == 62){
		  document.getElementById("fingering2").checked = true;
		  instrument = 2;
		 document.getElementById("instrument0").style.display= "none";
		document.getElementById("instrument1").style.display= "none";
		document.getElementById("instrument2").style.display= "block";
	
		document.getElementById("key0").style.display= "none";
		document.getElementById("key1").style.display= "none";
		document.getElementById("key2").style.display= "block";		  
		  advancedOkay(); //turn off the advanced tab	
		  updateCells();}
	  
	  if (event.data[2] == 70){document.getElementById("pitchbendradio0").checked = true;
	  updateCustom();
	  updateCustom();}
	  if (event.data[2] == 71){document.getElementById("pitchbendradio1").checked = true;
	  updateCustom();
	  updateCustom();}
	  if (event.data[2] == 72){document.getElementById("pitchbendradio2").checked = true;
	  updateCustom();
	  updateCustom();}
	  
	   if (event.data[2] == 80){document.getElementById("sensorradio0").checked = true;}
	  if (event.data[2] == 81){document.getElementById("sensorradio1").checked = true;}
	  if (event.data[2] == 82){document.getElementById("sensorradio2").checked = true;}
	   if (event.data[2] == 83){document.getElementById("sensorradio3").checked = true;}  
	   
	   
	   if (event.data[2] == 121){document.getElementById("bellSensor").style.opacity = 1;
	   		document.getElementById("1").disabled = false;
			document.getElementById("2").disabled = false;
			document.getElementById("v1").classList.add("sensorValueEnabled");
			
	   }  
	  if (event.data[2] == 120){document.getElementById("bellSensor").style.opacity = 0.1;
	  		document.getElementById("1").disabled = true;
			document.getElementById("2").disabled = true;
			document.getElementById("v1").classList.remove("sensorValueEnabled");
	  }  	   
   
	   for (var i=0; i< 8; i++) {	//update button configuration	   
		   if (event.data[2] == 90 + i){buttonRowWrite = i;}}
		   	   	  
		   
	   for (var j=0; j< 8; j++) {   //update button configuration	
			  if( buttonRowWrite == j){			
				  for (var k=0; k< 12; k++) {
				 		if (event.data[2] == (100 + k)){
							document.getElementById("row" + (buttonRowWrite)).value = k;}
						if (k < 5 && event.data[2] == 112 + k){document.getElementById("MIDIrow" + (buttonRowWrite)).value = k;	
						updateCells(); }}}} //update any dependant fields}						
				  
				  
		for (var l=0; l< 3; l++) {   //update momentary switches
			  if(buttonRowWrite == l){	  
			  	if (event.data[2] == 117){document.getElementById("checkbox" + l).checked = false;}
				if (event.data[2] == 118){document.getElementById("checkbox" + l).checked = true;}
				
				}}
	  

	  
  }
  
else if (event.data[1] == 103){document.getElementById("senseDistance").value = 0-event.data[2];}	  //set sensedistance  

else if (event.data[1] == 117){document.getElementById("depth").value = event.data[2] + 1;
  			var output = document.getElementById("demo14");
			output.innerHTML = event.data[2] + 1;}	  //set vibrato depth  
						
else if (event.data[1] == 104){
	jumpFactorWrite = event.data[2];
	} // so we know which pressure setting is going to be received.

else if (event.data[1] == 105 && jumpFactorWrite < 13){document.getElementById("jumpFactor" + jumpFactorWrite).value = event.data[2];
   			var output = document.getElementById("demo" + jumpFactorWrite);
			output.innerHTML = event.data[2];}
			
if (event.data[1] == 105) {
	

	if (jumpFactorWrite == 13){document.getElementById("checkbox6").checked = event.data[2];} 	 								 
	else if (jumpFactorWrite == 14){document.getElementById("expressionDepth").value = event.data[2];}
	else if (jumpFactorWrite == 15){document.getElementById("checkbox7").checked = event.data[2];} 	 								
	else if (jumpFactorWrite == 16 && event.data[2] == 0){document.getElementById("curveRadio0").checked = true;} 
	else if (jumpFactorWrite == 16 && event.data[2] == 1){document.getElementById("curveRadio1").checked = true;}
	else if (jumpFactorWrite == 16 && event.data[2] == 2){document.getElementById("curveRadio2").checked = true;}
	else if (jumpFactorWrite == 17){document.getElementById("pressureChannel").value = event.data[2];}
	else if (jumpFactorWrite == 18){document.getElementById("pressureCC").value = event.data[2];}
	else if (jumpFactorWrite == 19){slider.noUiSlider.set([event.data[2], null]);}
	else if (jumpFactorWrite == 20){slider.noUiSlider.set([null, event.data[2]]);}
	else if (jumpFactorWrite == 21){slider2.noUiSlider.set([event.data[2], null]);}
	else if (jumpFactorWrite == 22){slider2.noUiSlider.set([null, event.data[2]]);}
	else if (jumpFactorWrite == 23){document.getElementById("dronesOnCommand").value = event.data[2];}
	else if (jumpFactorWrite == 24){document.getElementById("dronesOnChannel").value = event.data[2];}
	else if (jumpFactorWrite == 25){document.getElementById("dronesOnByte2").value = event.data[2];}
	else if (jumpFactorWrite == 26){document.getElementById("dronesOnByte3").value = event.data[2];}	
	else if (jumpFactorWrite == 27){document.getElementById("dronesOffCommand").value = event.data[2];}
	else if (jumpFactorWrite == 28){document.getElementById("dronesOffChannel").value = event.data[2];}
	else if (jumpFactorWrite == 29){document.getElementById("dronesOffByte2").value = event.data[2];}
	else if (jumpFactorWrite == 30){document.getElementById("dronesOffByte3").value = event.data[2];}	
	else if (jumpFactorWrite == 31 && event.data[2] == 0){document.getElementById("dronesRadio0").checked = true;} 
	else if (jumpFactorWrite == 31 && event.data[2] == 1){document.getElementById("dronesRadio1").checked = true;}
	else if (jumpFactorWrite == 31 && event.data[2] == 2){document.getElementById("dronesRadio2").checked = true;}
	else if (jumpFactorWrite == 31 && event.data[2] == 3){document.getElementById("dronesRadio3").checked = true;}	
	else if (jumpFactorWrite == 32){lsb = event.data[2];}
	else if (jumpFactorWrite == 33){
		var x = parseInt((event.data[2] << 7) | lsb); //receive pressure between 100 and 900
		x= (x - 100) * 24 / 900; //convert to inches of water. 105 is the approximate minimum sensor value.
		var p = x.toFixed(1); //round to 1 decimal
		p = Math.min(Math.max(p, 0), 24); //constrain
		document.getElementById("dronesPressureInput").value = p;}
		else if (jumpFactorWrite == 34){lsb = event.data[2];

	}
		
	else if (jumpFactorWrite == 35){
		var x = parseInt((event.data[2] << 7) | lsb); //receive pressure between 100 and 900
		x= (x - 100) * 24 / 900; //convert to inches of water.  105 is the approximate minimum sensor value.
		var p = x.toFixed(1); //round to 1 decimal
		p = Math.min(Math.max(p, 0), 24); //constrain
		document.getElementById("octavePressureInput").value = p;}
	
	else if (jumpFactorWrite == 43){document.getElementById("checkbox9").checked = event.data[2];} 	 //invert											
	else if (jumpFactorWrite == 44){document.getElementById("checkbox5").checked = event.data[2]; //custom
			updateCustom();
	} 	 										
	else if (jumpFactorWrite == 42){document.getElementById("checkbox3").checked = event.data[2];} 	 //secret										
	else if (jumpFactorWrite == 40){document.getElementById("checkbox4").checked = event.data[2];} 	 //vented							 			
	else if (jumpFactorWrite == 41){document.getElementById("checkbox8").checked = event.data[2];} 	 //bagless						 	




	
	}
	
			
//else if (event.data[1] == 109){document.getElementById("hysteresis").value = event.data[2];
   			//var output = document.getElementById("demo13");
			//output.innerHTML = event.data[2];}  	
else if (event.data[1] == 110){    	
			version = event.data[2];
			

						if (version == currentVersion){
							
			document.getElementById("current").innerHTML = "Your software is up to date.";
			document.getElementById("current").style.left = "710px";
			document.getElementById("current").style.visibility = "visible";
			document.getElementById("status").style.visibility = "hidden";
			document.getElementById("current").style.color = "#f7c839";
			}
			
			
			else{
			document.getElementById("current").innerHTML = "There is a software update available.";
			document.getElementById("current").style.left = "690px";
			document.getElementById("current").style.visibility = "visible";
						document.getElementById("status").style.visibility = "hidden";
			document.getElementById("current").style.color = "#F78339";
			}
			

			version = version/10;
			document.getElementById("version").innerHTML = version;
			document.getElementById("version").style.color = "#f7c839";
			
			
			}
			
else if (event.data[1] == 111){
	document.getElementById("keySelect0").value = event.data[2];}
 else if (event.data[1] == 112){
	document.getElementById("keySelect1").value = event.data[2];}
 else if (event.data[1] == 113){
	document.getElementById("keySelect2").value = event.data[2];}		
		
else if (event.data[1] == 116){
	lsb = event.data[2];}
	
else if (event.data[1] == 118){	
	var x = parseInt((event.data[2] << 7) | lsb); //receive pressure between 100 and 900
	x= (x - 100) * 24 / 900; //convert to inches of water.  105 is the approximate minimum sensor value.
	p = Math.min(Math.max(p, 0), 24); //constrain
	var p = x.toFixed(1); //round to 1 decimal
	if(p < 0.2) {p = 0};
	document.getElementById("pressure").innerHTML = (p);
	document.getElementById("pressure1").innerHTML = (p);}
	
			
for (var i=0; i< 8; i++) {
	  if(buttonRowWrite == i){
		if (event.data[1] == 106 && event.data[2] < 16){document.getElementById("channel" + (buttonRowWrite)).value = event.data[2];	}
		if (event.data[1] == 107){document.getElementById("byte2_" + (buttonRowWrite)).value = event.data[2];	}
		if (event.data[1] == 108){document.getElementById("byte3_" + (buttonRowWrite)).value = event.data[2];	}}}
		
if (event.data[1] == 106 && event.data[2] > 15){


	
	if(event.data[2] > 19 && event.data[2] < 29){
		document.getElementById("vibratoCheckbox" + (event.data[2]-20)).checked = false;
	}
		
	if(event.data[2] > 29 && event.data[2] < 39){
		document.getElementById("vibratoCheckbox" + (event.data[2]-30)).checked = true;
	}
		
	if (event.data[2] == 39){document.getElementById("calibrateradio0").checked = true;}
	if (event.data[2] == 40){document.getElementById("calibrateradio1").checked = true;}	
	

}
				
		}
}
	updateCells(); //keep enabled/disabled cells updated.
}



	

function bit_test(num, bit){
    return ((num>>bit) % 2 != 0)
}


function sendFingeringSelect(row) {
	updateCells();	
	updateCustom();
	blink(1);
	var x = parseFloat(document.getElementById("fingeringSelect" + row).value);
	//default keys
	if(x==2) {document.getElementById("keySelect" + row).value = 8;} //GHB
	else if(x==3) {document.getElementById("keySelect" + row).value = 3;} //Northumbrian
	else if(x==5) {document.getElementById("keySelect" + row).value = 125;} //Gaita
	else if(x==6) {document.getElementById("keySelect" + row).value = 122;} //NAF
	else if(x==8) {document.getElementById("keySelect" + row).value = 125;} //Recorder
	else {document.getElementById("keySelect" + row).value = 0;} //default key of D for many patterns


	//send the fingering pattern
	send(102,30 + row);	
	var y = parseFloat(document.getElementById("fingeringSelect" + row).value);
	send(102,33 + y);

	
	sendKey(row);
}
	

function sendKey(row) {
	updateCells();
	blink(1);
	var y = parseFloat(document.getElementById("keySelect" + row).value);
	send(111 + row,y);}	
	
function sendFingeringRadio() {	//change instruments, showing the correct tab for each instrument.

	updateCustom();
	advancedOkay(); //turn off the advanced tab	
	
	if(document.getElementById("fingering0").checked == true){
	instrument = 0;
	document.getElementById("instrument0").style.display= "block";
	document.getElementById("instrument1").style.display= "none";
	document.getElementById("instrument2").style.display= "none";
	
	document.getElementById("key0").style.display= "block";
	document.getElementById("key1").style.display= "none";
	document.getElementById("key2").style.display= "none";
	send(102,60);
	
	}
	else if(document.getElementById("fingering1").checked == true){
	instrument = 1;
	document.getElementById("instrument0").style.display= "none";
	document.getElementById("instrument1").style.display= "block";
	document.getElementById("instrument2").style.display= "none";
	
	document.getElementById("key0").style.display= "none";
	document.getElementById("key1").style.display= "block";
	document.getElementById("key2").style.display= "none";
		blink(2);
	send(102,61);}
	else if(document.getElementById("fingering2").checked == true){
	instrument = 2;
	document.getElementById("instrument0").style.display= "none";
	document.getElementById("instrument1").style.display= "none";
	document.getElementById("instrument2").style.display= "block";
	
			document.getElementById("key0").style.display= "none";
	document.getElementById("key1").style.display= "none";
	document.getElementById("key2").style.display= "block";
	blink(3);
	send(102,62);}
	updateCells();
	updateCustom();
	}
	
function sendSenseDistance() {
	blink(1);
	var x = parseFloat(document.getElementById("senseDistance").value);	
	x = 0-x;
	send(103,x);}
	
	
function sendDepth() {
	blink(1);
	var x = parseFloat(document.getElementById("depth").value);	
	send(117,x);
}


function sendExpressionDepth() {
	blink(1);
	var x = parseFloat(document.getElementById("expressionDepth").value);	
	send(104,14);
	send(105,x);
}

function sendRawPressure(){
	blink(1);
	if(document.getElementById("checkbox7").checked == true){
	send(104,15);
	send(105,1);
	}
	else{
		send(104,15);
		send(105,0);
		}
	}

function sendCurveRadio() {
	blink(1);
	send(104,16);	
	if(document.getElementById("curveRadio0").checked == true){
	send(105,0);		
	}
	else if(document.getElementById("curveRadio1").checked == true){
	send(105,1);		
	}
	else if(document.getElementById("curveRadio2").checked == true){
	send(105,2);		
	}			
}


function sendPressureChannel(){
	blink(1);
	var y = parseFloat(document.getElementById("pressureChannel").value);
		send(104,17);
		send(105,y);
}

function sendPressureCC(){
	blink(1);
	var y = parseFloat(document.getElementById("pressureCC").value);
		send(104,18);
		send(105,y);
}


//pressure input slider
slider.noUiSlider.on('change', function (values) {
		blink(1);
		send(104,19);
		send(105,parseInt(values[0]));
		send(104,20);
		send(105,parseInt(values[1]));
});

//pressure output slider
slider2.noUiSlider.on('change', function (values) {
		blink(1);
		send(104,21);
		send(105,parseInt(values[0]));
		send(104,22);
		send(105,parseInt(values[1]));
});


function sendDronesOnCommand(){
		blink(1);
		var y = parseFloat(document.getElementById("dronesOnCommand").value);
		send(104,23);
		send(105,y);
}

function sendDronesOnChannel(){
		var x = parseFloat(document.getElementById("dronesOnChannel").value);
		   if (x<0 || x>16 || isNaN(x))  {
   alert("Value must be 1-16.");
   	document.getElementById("dronesOnChannel").value = null;}
   	else{
   			blink(1);
		send(104,24);
		send(105,x);
}}


function sendDronesOnByte2(){
		var x = parseFloat(document.getElementById("dronesOnByte2").value);
		   if (x<0 || x>127 || isNaN(x))  {
   alert("Value must be 0 - 127.");
   	document.getElementById("dronesOnByte2").value = null;}
   	else{
   		blink(1);
		send(104,25);
		send(105,x);
}}


function sendDronesOnByte3(){
		var x = parseFloat(document.getElementById("dronesOnByte3").value);
		   if (x<0 || x>127 || isNaN(x))  {
   alert("Value must be 0 - 127.");
   	document.getElementById("dronesOnByte3").value = null;}
   	else{
   		blink(1);
		send(104,26);
		send(105,x);
}}


function sendDronesOffCommand(){
		blink(1);
		var y = parseFloat(document.getElementById("dronesOffCommand").value);
		send(104,27);
		send(105,y);
}


function sendDronesOffChannel(){
		var x = parseFloat(document.getElementById("dronesOffChannel").value);
		   if (x<0 || x>16 || isNaN(x))  {
   alert("Value must be 1-16.");
   	document.getElementById("dronesOffChannel").value = null;}
   	else{
   			blink(1);
		send(104,28);
		send(105,x);
}}


function sendDronesOffByte2(){
		var x = parseFloat(document.getElementById("dronesOffByte2").value);
		   if (x<0 || x>127 || isNaN(x))  {
   alert("Value must be 0 - 127.");
   	document.getElementById("dronesOffByte2").value = null;}
   	else{
   		blink(1);
		send(104,29);
		send(105,x);
}}


function sendDronesOffByte3(){
		var x = parseFloat(document.getElementById("dronesOffByte3").value);
		   if (x<0 || x>127 || isNaN(x))  {
   alert("Value must be 0 - 127.");
   	document.getElementById("dronesOffByte3").value = null;}
   	else{
   		blink(1);
		send(104,30);
		send(105,x);
}}


function sendDronesRadio() {
	blink(1);
	send(104,31);	
	if(document.getElementById("dronesRadio0").checked == true){
	send(105,0);		
	}
	else if(document.getElementById("dronesRadio1").checked == true){
	send(105,1);		
	}
	else if(document.getElementById("dronesRadio2").checked == true){
	send(105,2);
	}
	else if(document.getElementById("dronesRadio3").checked == true){
	send(105,3);
		
	}			

}

function learnDrones() {
	blink(1);
	document.getElementById("dronesPressureInput").style.backgroundColor = "#32CD32";
	setTimeout(blinkDrones, 500);
	send(106,43);
	
}

function blinkDrones() {
document.getElementById("dronesPressureInput").style.backgroundColor = "#FFF";
}


function blinkOctave() {
document.getElementById("octavePressureInput").style.backgroundColor = "#FFF";
}

function sendDronesPressure() {
	var x = document.getElementById("dronesPressureInput").value;
	if (x<0 || x>24 || isNaN(x))  {
   	 alert("Value must be 0.0 - 24.0.");
  	  document.getElementById("dronesPressureInput").value = null;}
	else{
	x = parseInt(x * 900 / 24 + 100);
	blink(1);
	document.getElementById("dronesPressureInput").style.backgroundColor = "#32CD32";
	setTimeout(blinkDrones, 500);		
	send(104,32);
	send(105, x  & 0x7F);
	send(104,33);
	send(105, x >> 7);
	}

}


function sendOctavePressure() {
	var x = document.getElementById("octavePressureInput").value;
	if (x<0 || x>24 || isNaN(x))  {
    	alert("Value must be 0.0 - 24.0.");
        document.getElementById("octavePressureInput").value = null;} 
	else {
	x = parseInt(x * 900 / 24 + 100);
	blink(1);
	document.getElementById("octavePressureInput").style.backgroundColor = "#32CD32";
	setTimeout(blinkOctave, 500);		
	send(104,34);
	send(105, x  & 0x7F);
	send(104,35);
	send(105, x >> 7);}

}


function learn() {
	blink(1);
	document.getElementById("octavePressureInput").style.backgroundColor = "#32CD32";
	setTimeout(blinkOctave, 500);
	send(106,41);
}


function sendCalibrateRadio() {
	blink(1);
	if(document.getElementById("calibrateradio0").checked == true){
	send(106,39);	
	}
	else if(document.getElementById("calibrateradio1").checked == true){
	send(106,40);}
	}
	

	
	
function sendPitchbendRadio() {
	updateCustom();	
	updateCustom();
	if(document.getElementById("pitchbendradio0").checked == true){
	send(102,70);	
	}
	else if(document.getElementById("pitchbendradio1").checked == true){
		send(102,71);
		blink(2);}
	else if(document.getElementById("pitchbendradio2").checked == true){
		blink(3);
		send(102,72);}}

	
	
function sendExpression() {
		blink(1);
		if(document.getElementById("checkbox6").checked == true){
		send(104,13);
		send(105,1);
	}
	else{
		send(104,13);
		send(105,0);
		}
	}
	
	
	
function sendVibratoHoles(holeNumber)  {
	if(document.getElementById("vibratoCheckbox" + holeNumber).checked == true){
		send(106,20 + holeNumber);
	}
	
	if(document.getElementById("vibratoCheckbox" + holeNumber).checked == false){
		send(106,30 + holeNumber);
	}
	
}
	
	
	
	
function updateCustom() { //keep correct settings enabled/disabled with respect to the custom vibrato switch.
var a = document.getElementById("fingeringSelect" + instrument).value;

			
		if(document.getElementById("pitchbendradio1").checked == true && (a == 0 || a == 1 || a == 4)){
				document.getElementById("checkbox5").disabled = false;
				}
		else{document.getElementById("checkbox5").disabled = true;}
		if(document.getElementById("checkbox5").checked == true && document.getElementById("checkbox5").disabled == false){
			for (var i=0; i<9; i++) {
				document.getElementById("vibratoCheckbox" + i).disabled = true;
				document.getElementById("vibratoCheckbox" + i).style.cursor = "default";
			}
		}
		else{
			for (var i=0; i<9; i++) {
				document.getElementById("vibratoCheckbox" + i).disabled = false;
				document.getElementById("vibratoCheckbox" + i).style.cursor = "pointer";
		}
			}	
}
	
	
	
	
function sendBreathmodeRadio() {		
	if(document.getElementById("sensorradio0").checked == true){
	send(102,80);}
	else if(document.getElementById("sensorradio1").checked == true){
		blink(2);
	send(102,81);}
	else if(document.getElementById("sensorradio2").checked == true){
		blink(3);
	send(102,82);}
	else if(document.getElementById("sensorradio3").checked == true){
		blink(4);
	send(102,83);}
	else if(document.getElementById("sensorradio4").checked == true){
		blink(5);
	send(102,84);}	
}
	
	
function advanced() {
	document.getElementById("box2").style.display = "block";
	document.getElementById("box1").style.display = "none";
}

function advancedOkay() {
	document.getElementById("box2").style.display = "none";
	document.getElementById("box1").style.display = "block";
}

function advancedDefaults(){
	document.getElementById("jumpFactor1").value = "25";
	document.getElementById("jumpFactor2").value = "15";
	document.getElementById("jumpFactor3").value = "15";
	document.getElementById("jumpFactor4").value = "15";
	document.getElementById("jumpFactor5").value = "30";
	document.getElementById("jumpFactor6").value = "60";
	document.getElementById("jumpFactor7").value = "3";
	document.getElementById("jumpFactor8").value = "8";
	document.getElementById("jumpFactor9").value = "28";
	document.getElementById("jumpFactor10").value = "7";
	document.getElementById("jumpFactor11").value = "15";
	document.getElementById("jumpFactor12").value = "22";
	updateSliders();	
	
	for (var i=1; i< 13; i++) {
	sendJumpFactor(i);}
}
	
	
function advancedBagDefaults(){
	document.getElementById("jumpFactor1").value = "75";
	document.getElementById("jumpFactor2").value = "25";
	document.getElementById("jumpFactor3").value = "15";
	document.getElementById("jumpFactor4").value = "15";
	document.getElementById("jumpFactor5").value = "30";
	document.getElementById("jumpFactor6").value = "60";
	document.getElementById("jumpFactor7").value = "50";
	document.getElementById("jumpFactor8").value = "20";
	document.getElementById("jumpFactor9").value = "28";
	document.getElementById("jumpFactor10").value = "7";
	document.getElementById("jumpFactor11").value = "15";
	document.getElementById("jumpFactor12").value = "22";
	updateSliders();	
	
	for (var i=1; i< 13; i++) {
	sendJumpFactor(i);}
}	
	
	
	
	
	
function sendJumpFactor(factor) {
	blink(1);
	var x = parseFloat(document.getElementById("jumpFactor" + factor).value);
	updateSliders();	
	send(104,factor);	
	send(105,x);}
	

//function sendHysteresis() {
	//blink(1);
	//var x = parseFloat(document.getElementById("hysteresis").value);	
	//send(109,x);}

function sendRow(rowNum) {
	blink(1);
	updateCells();
	send(102,90 + rowNum);
	var y = (100)+ parseFloat(document.getElementById("row" + rowNum).value);
	send(102,y);
	sendMIDIrow(rowNum);
	sendChannel(rowNum);
	sendByte2(rowNum);
	sendByte3(rowNum);
	if(rowNum<3) {sendMomentary(rowNum);}}
	
function sendMIDIrow(MIDIrowNum) {
	blink(1);
	updateCells();
	send(102,90 + MIDIrowNum);
	var y = (112)+ parseFloat(document.getElementById("MIDIrow" + MIDIrowNum).value);
	send(102,y);
	sendChannel(MIDIrowNum);
	sendByte2(MIDIrowNum);
	sendByte3(MIDIrowNum);
	if(MIDIrowNum<3) {sendMomentary(MIDIrowNum);}}	

function sendChannel(rowNum) {
	blink(1);
	MIDIvalueChange();
	var y = parseFloat(document.getElementById("channel" + (rowNum)).value);
	send(102,90 + rowNum);
	send(106,y);}			

function sendByte2(rowNum) {
	blink(1);
	MIDIvalueChange();
	send(102,90 + rowNum);
	var y = parseFloat(document.getElementById("byte2_" + (rowNum)).value); 
	send(107,y);}	
	
function sendByte3(rowNum) {
	blink(1);
	MIDIvalueChange();
	send(102,90 + rowNum);
	var y = parseFloat(document.getElementById("byte3_" + (rowNum)).value);
	send(108,y);}	

function sendMomentary(rowNum) { //send momentary
	blink(1);
	updateCells();
	var y = document.getElementById("checkbox" + rowNum).checked
	send(102,90 + rowNum);
	if (y==false){send(102,117);}
	if (y==true){send(102,118);}}
	
	
	
	
function sendVented() {
		blink(1);
		if(document.getElementById("checkbox4").checked == true){
		send(104,40);
		send(105,1);
	}
	else{
		send(104,40);
		send(105,0);
		}
	}	
	
function sendBagless() {
		blink(1);
		if(document.getElementById("checkbox8").checked == true){
		send(104,41);
		send(105,1);
	}
	else{
		send(104,41);
		send(105,0);
		}
	}	
	
	
function sendSecret() {
		blink(1);
		if(document.getElementById("checkbox3").checked == true){
		send(104,42);
		send(105,1);
	}
	else{
		send(104,42);
		send(105,0);
		}
	}		
	
	
function sendInvert() {
		blink(1);
		if(document.getElementById("checkbox9").checked == true){
		send(104,43);
		send(105,1);
	}
	else{
		send(104,43);
		send(105,0);
		}
	}	
	
	
function sendCustom() {
		blink(1);
		updateCustom();
		if(document.getElementById("checkbox5").checked == true){
		send(104,44);
		send(105,1);
	}
	else{
		send(104,44);
		send(105,0);
		}
	
	}
	
	
function saveAsDefaults() {
	modalclose(2);
	blink(3);
	send(106,45);}
	
function saveAsDefaultsForAll() {
	modalclose(3);
	blink(3);
	send(102,124);}
	
function restoreAll() {
	modalclose(4);
	blink(3);
	send(102,125);}
	
function autoCalibrateBell() {
		LEDon();
	 	setTimeout(LEDoff, 5000);
		send(106,42);}
	
function autoCalibrate() {
	 	LEDon();
	 	setTimeout(LEDoff, 10000);
		send(102,127);}

function frequencyFromNoteNumber(note) {
  return 440 * Math.pow(2,(note-69)/12);}

function modal(modalId) {	
	document.getElementById("open-modal" +  modalId).classList.add('modal-window-open');}

function modalclose(modalId) {
document.getElementById("open-modal" +  modalId).classList.remove('modal-window-open');}

function send(byte2,byte3) {
		if (!(byte2==102 && byte3==127) && !(byte2==106 && byte3==42)) {blink(1);} //blink once if we aren't doing autocalibration, which requires the LED to be onl longer.
		
	if(byte2==102) { 
		if(byte3==19) {  //send message to save sensor calibration
			blink(3);
			for (var i = 1; i < 10; i++) {			
			document.getElementById("v"+(i)).innerHTML = "0";}
		for (var i = 0; i < 19; i++) {			
			sensorValue[i] = 0;}}
					
		
		if(isEven(byte3)&& byte3 < 19){ //send sensor calibration values
			sensorValue[byte3-2] ++;
			document.getElementById("v"+(byte3>>1)).innerHTML = sensorValue[byte3-2];}
			
			
		if(isOdd(byte3)&& byte3 < 19){
			sensorValue[byte3-1] --;
			document.getElementById("v"+((byte3 + 1)>>1)).innerHTML = sensorValue[byte3-1];
			checkMax((byte3 + 1)>>1);}
		}
				
	 cc = [0xB6, byte2, byte3]; //prepare message
       var iter = midiAccess.outputs.values();
       for (var o = iter.next(); !o.done; o = iter.next()) {
        o.value.send(cc , window.performance.now() ); //send CC message
        }
		}
		

function blink(blinkNum) {
	 LEDon();
	 	setTimeout(LEDoff, 200);
	 if(blinkNum > 1){ //blink twice
	 	 setTimeout(LEDon, 400);
	 	setTimeout(LEDoff, 600);}
	 if(blinkNum > 2){ // blink thrice
	 	 setTimeout(LEDon, 800);
	 	setTimeout(LEDoff, 1000);}
	 if(blinkNum > 3){ //blink four times
	 	 setTimeout(LEDon, 1200);
	 	setTimeout(LEDoff, 1400);}
	 if(blinkNum > 4){ //blink five times
	 	 setTimeout(LEDon, 1600);
	 	setTimeout(LEDoff, 1800);} 	
	 	}


function checkMax(field) { //disable the sensor value field if it has been set to MAX
	var x = document.getElementById("v" + field).textContent;
	//console.log(x);
	if (x == "MAX"){document.getElementById(field * 2).disabled = true;}
	if (x != "MAX"){document.getElementById(field * 2).disabled = false;}}


function isEven(n) {
   return n % 2 == 0;}
   

function isOdd(n) {
   return Math.abs(n % 2) == 1;}
   

function LEDoff()  {
	//console.log("off");
	elements = document.getElementsByClassName("leddot");
    for (var i = 0; i < elements.length; i++) {
        elements[i].style.visibility = "hidden";}}
	 

function LEDon()  {
	elements = document.getElementsByClassName("leddot");
    for (var i = 0; i < elements.length; i++) {
        elements[i].style.visibility = "visible";}}


function toggleOn(){
	volume = +!volume;
	if( volume == 0){
		noteOff(currentNote)}}


function noteOn(noteNumber) {
	currentNote = noteNumber;
	if(volume == 1){
	                var oscillator = context.createOscillator();
                oscillator.type = 'square';
                oscillator.frequency.value = frequencyFromNoteNumber(noteNumber);
                oscillator.connect(masterGain);
                oscillator.start(0);
                nodes.push(oscillator);}}


function noteOff(noteNumber) {
	        var new_nodes = [];

                for (var i = 0; i < nodes.length; i++) {
                    if (Math.round(nodes[i].frequency.value) === Math.round(frequencyFromNoteNumber(noteNumber))) {
                        nodes[i].stop(0);
                        nodes[i].disconnect(); } 
                    else { new_nodes.push(nodes[i]); }
                }
                nodes = new_nodes;}

function updateSliders() {

var slider1 = document.getElementById("jumpFactor1");
var output1 = document.getElementById("demo1");
output1.innerHTML = slider1.value;
slider1.oninput = function() {
  output1.innerHTML = this.value;}

var slider2 = document.getElementById("jumpFactor2");
var output2 = document.getElementById("demo2");
output2.innerHTML = slider2.value;
slider2.oninput = function() {
  output2.innerHTML = this.value;}
  
var slider3 = document.getElementById("jumpFactor3");
var output3 = document.getElementById("demo3");
output3.innerHTML = slider3.value;
slider3.oninput = function() {
  output3.innerHTML = this.value;}
  
var slider4 = document.getElementById("jumpFactor4");
var output4 = document.getElementById("demo4");
output4.innerHTML = slider4.value;
slider4.oninput = function() {
  output4.innerHTML = this.value;}
  
var slider5 = document.getElementById("jumpFactor5");
var output5 = document.getElementById("demo5");
output5.innerHTML = slider5.value;
slider5.oninput = function() {
  output5.innerHTML = this.value;}
  
var slider6 = document.getElementById("jumpFactor6");
var output6 = document.getElementById("demo6");
output6.innerHTML = slider6.value;
slider6.oninput = function() {
  output6.innerHTML = this.value;}
  
var slider7 = document.getElementById("jumpFactor7");
var output7 = document.getElementById("demo7");
output7.innerHTML = slider7.value;
slider7.oninput = function() {
  output7.innerHTML = this.value;}
  
var slider8 = document.getElementById("jumpFactor8");
var output8 = document.getElementById("demo8");
output8.innerHTML = slider8.value;
slider8.oninput = function() {
  output8.innerHTML = this.value;}
  

var slider9 = document.getElementById("jumpFactor9");
var output9 = document.getElementById("demo9");
output9.innerHTML = slider9.value;
slider9.oninput = function() {
  output9.innerHTML = this.value;}
  

var slider10 = document.getElementById("jumpFactor10");
var output10 = document.getElementById("demo10");
output10.innerHTML = slider10.value;
slider10.oninput = function() {
  output10.innerHTML = this.value;}
  

var slider11 = document.getElementById("jumpFactor11");
var output11 = document.getElementById("demo11");
output11.innerHTML = slider11.value;
slider11.oninput = function() {
  output11.innerHTML = this.value;}
  

var slider12 = document.getElementById("jumpFactor12");
var output12 = document.getElementById("demo12");
output12.innerHTML = slider12.value;
slider12.oninput = function() {
  output12.innerHTML = this.value;}  
  
  
  
//var slider13 = document.getElementById("hysteresis");
//var output13 = document.getElementById("demo13");
//output13.innerHTML = slider13.value;
//slider13.oninput = function() {
  //output13.innerHTML = this.value;} 
  
  
var slider14 = document.getElementById("depth");
var output14 = document.getElementById("demo14");
output14.innerHTML = slider14.value;
slider14.oninput = function() {
  output14.innerHTML = this.value;} 
  

}



//sets up initial values for selects/fields/radios and constantly keeps the proper options enabled/disabled
function updateCells() {  


 var q = document.getElementById("checkbox0").checked;
 	if (q == true) {document.getElementById("row5").disabled = true;}
 	else{document.getElementById("row5").disabled = false;}
 	
 var r = document.getElementById("checkbox1").checked;
 	if (r == true) {document.getElementById("row6").disabled = true;}
 	 	else {document.getElementById("row6").disabled = false;}
 	
 	if (q == true || r == true){
 	document.getElementById("row3").disabled = true;}
 	 	 	else {document.getElementById("row3").disabled = false;}
 	
 var s = document.getElementById("checkbox2").checked;
 	if (s == true) {document.getElementById("row7").disabled = true;}
 	else{document.getElementById("row7").disabled = false;}
 	
 	if (r == true || s == true){
 	document.getElementById("row4").disabled = true;}
 	 	 	else {document.getElementById("row4").disabled = false;}


		
for (var i = 0; i < 8; i++) {
    var x = document.getElementById("row" + i).value;
    var t = document.getElementById("row" + i).disabled;
    
    if ((x!=1 || t == true)) {document.getElementById("MIDIrow" + i).disabled = true;
    document.getElementById("channel" + i).disabled = true; 
    document.getElementById("byte2_" + i).disabled = true; 
    document.getElementById("byte3_" + i).disabled = true; 
   }
    else {document.getElementById("MIDIrow" + i).disabled = false; 
   document.getElementById("channel" + i).disabled = false; 
      document.getElementById("byte2_" + i).disabled = false; 
    document.getElementById("byte3_" + i).disabled = false;   }
 
 
     var y = document.getElementById("MIDIrow" + i).value;
    if (y==2) {document.getElementById("byte3_" + i).disabled = true;}
	if (y > 2) {
		document.getElementById("byte2_" + i).disabled = true;
		document.getElementById("byte3_" + i).disabled = true;
 }
	
	
	 if (y==0 && i < 3 && x==1) {
		document.getElementById("checkbox" + i).disabled = false;
		document.getElementById("switch" + i).style.cursor = "pointer";}
	
	
   	if ((y!=0 && i < 3) || (i < 3 && (x==0 || x > 1))) {
		document.getElementById("checkbox" + i).disabled = true;
		document.getElementById("switch" + i).style.cursor = "default";
		document.getElementById("checkbox" + i).checked = false;}
	
	
	
 
  var q = document.getElementById("checkbox0").checked;
 	if (q == true) {document.getElementById("row5").disabled = true;}
 	else{document.getElementById("row5").disabled = false;}
 	
 var r = document.getElementById("checkbox1").checked;
 	if (r == true) {document.getElementById("row6").disabled = true;}
 	 	else {document.getElementById("row6").disabled = false;}
 	
 	if (q == true || r == true){
 	document.getElementById("row3").disabled = true;}
 	 	 	else {document.getElementById("row3").disabled = false;}
 	
 var s = document.getElementById("checkbox2").checked;
 	if (s == true) {document.getElementById("row7").disabled = true;}
 	else{document.getElementById("row7").disabled = false;}
 	
 	if (r == true || s == true){
 	document.getElementById("row4").disabled = true;}
 	 	 	else {document.getElementById("row4").disabled = false;}
 
 
 
}

 	
 
 var z = $("#fingeringSelect0 option:selected").text();
     document.getElementById("fingeringLabel1").innerHTML = z;
    
 var z = $("#fingeringSelect1 option:selected").text();
     document.getElementById("fingeringLabel2").innerHTML = z;
     
 var z = $("#fingeringSelect2 option:selected").text();
     document.getElementById("fingeringLabel3").innerHTML = z;
	 
	 

}



function MIDIvalueChange() {
for (var i = 0; i < 8; i++) {
    var x = document.getElementById("channel" + i).value;
   if (((x<0 || x>16 || isNaN(x))&& !document.getElementById("channel" + i).disabled))  {
   alert("Value must be 1-16.");
   	document.getElementById("channel" + i).value = null;
   }
	}
for (var j = 0; j < 8; j++) {
	var y = document.getElementById("channel" + j).value;
    var x = document.getElementById("byte2_" + j).value;
   if (x<0 || x>127 || isNaN(x)) {
   alert("Value must be 0-127.");
   	document.getElementById("byte2_" + j).value = null;}
	   if (y ==7 && x>101 && x<120) {
   alert("This CC range on channel 7 is reserved for the Configuration Tool.");
   	document.getElementById("byte2_" + j).value = null;
	
	   }
	}

for (var k = 0; k < 8; k++) {
    var x = document.getElementById("byte3_" + k).value;
   if (x<0 || x>127 || isNaN(x)) {
   alert("Value must be 0-127.");
   	document.getElementById("byte3_" + k).value = null;
   }
	}
	
}




            window.AudioContext = window.AudioContext || window.webkitAudioContext;
            var context = new AudioContext(),
            masterGain = context.createGain();
            nodes = [];
            masterGain.gain.value = 0.3;
            masterGain.connect(context.destination); 





/* Toggle between adding and removing the "responsive" class to topnav when the user clicks on the icon */
function topNavFunction() {
    var x = document.getElementById("myTopnav");
    if (x.className === "topnav") {
        x.className += " responsive";
    } else {
        x.className = "topnav";
    }
}









        

