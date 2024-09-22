
// WARBL Web Configuration Tool MIDI Support Routines
//

//debugger;
var sendDelay = 15 //milliseconds delay between sending commands to the WARBL

var mapSelection; //keeps track of which pressure output we're mapping (CC, vel, aftertouch, poly)
var IMUmapSelection; //keeps track of which IMU output we're mapping (roll, pitch, yaw)
var center = [1, 1, 1, 1]; //center range of input sliders for IMU mapping
var IMUchannel = [1, 1, 1]; //CC channels for roll, pitch, yaw mapping
var IMUnumber = [2, 2, 2]; //CC numbers for roll, pitch, yaw mapping
var curve = [0, 0, 0, 0]; //which curve is selected for CC, vel, aftertouch, poly
var inputSliderMin = [0, 0, 0, 0, -90, -90, -90]; //settings for pressure and IMU sliders
var inputSliderMax = [100, 100, 100, 100, 90, 90, 90];
var outputSliderMin = [0, 0, 0, 0, 0, 0, 0];
var outputSliderMax = [127, 127, 127, 127, 127, 127, 127];
var consoleEntries = 0; //number of lines in MIDI console
var customFingeringFills = [[null, null, null, null, null, null, null, null, null, null, null], [0, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61], [0, 74, 72, 72, 70, 68, 67, 65, 64, 62, 61], [0, 74, 74, 72, 72, 69, 68, 67, 65, 62, 60]];
var communicationMode = false; //if we're communicating with WARBL
var noteNames = ["C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C","C#","D","Eb","E","F","F#","G"];
var noteNamesforKeySelect = ["G#","A","Bb","B","C3","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C4","C#","D","Eb","E","F","F#","G","G#","A","Bb","B","C5","C#","D","Eb","F","F#","G","G#","A"];

var notePlaying = 0; //MIDI note most recently turned on, used for small note display
var numberOfGestures = 10; //Number of button gestures

var midiNotes = [];

var currentVersion = 42; // Current version of the WARBL2 firmware
var currentVersionOriginal = 21; // Current version of the original WARBL firmware
var previousVersion = 0; //Used to keep track of which version of WARBL is connected so that the page will refresh if a different WARBL with an older firmware version is connected.

var midiAccess = null; // the MIDIAccess object.

var WARBLout = null; // WARBL output port
var WARBLin = null; // WARBL input port
var midiConnecting = false;
var midiScanningOut = null;
var selectedMidiOutId = 0;

var gExportProxy = null; // Export received message proxy

// For pressure graphing
var gPressureGraphEnabled = false; // When true, proxy pressure messages to graphing code
var gCurrentPressure = 0.0;
var gGraphInterval = null;
var gNPressureReadings = 100;
var gMaxPressure = 25;
var gPressureReadings = [];

var on, off;

var deviceName;

var volume = 0;

var currentNote = 62;

var sendAllQueue = []; //queue for outgoing messages to WARBL port
var sendWARBLoutQueue = []; //queue for outgoing messages to all ports
var sendWARBLInterval = setInterval(sendQueuedWARBLoutMessages, sendDelay); //start sending any queued messages 
var sendAllInterval = setInterval(sendAllQueuedMessages, sendDelay); //start sending any queued messages

var sensorValue = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

var buttonRowWrite; //used to indicate which button command row we'll be writing to below.

var jumpFactorWrite; //to indicate which pressure variable is going to be sent or received

var fingeringWrite; //indicates the instrument for which a fingering pattern is being sent.

var WARBL2SettingsReceive; //indicates which WARBL2 setting is about to be received on CC119

var version = "Unknown";

var instrument = 0; //currently selected instrument tab

var lsb = 0; //used for reassembling pressure bytes from WARBL

var connIntvlLSB = 0; //used for reassembling connection interval from WARBL

var defaultInstr = 0; //default instrument

var modals = new Array();

for (var w = 1; w <= 29; w++) {

    modals[w] = document.getElementById("open-modal" + w);

}

updatePlatform();


// When the user clicks anywhere outside of the modal, close it
window.onclick = function (event) {

    for (var i = 1; i <= 30; i++) {

        if (event.target == modals[i]) {

            modalclose(i);

        }
    }
    event.stopPropagation();
}


//console.log('Initially ' + (window.navigator.onLine ? 'on' : 'off') + 'line'); // Detect if there's a connection-- can be used for refreshing the app version if a connection is available?


window.addEventListener('load', function () {

    //console.log("window onLoad");

    // Clear the WARBL output port
    WARBLout = null;
    WARBLin = null;

    updateCells(); // set selects and radio buttons to initial values and enabled/disabled states

    for (var i = 1; i < 13; i++) { //trigger input event for sliders to display values
        var k = document.getElementById("jumpFactor" + (i));
        k.dispatchEvent(new Event('input'));
        if (i == 10) {
            var k = document.getElementById("jumpFactor10b"); //special case for key delay slider
            k.dispatchEvent(new Event('input'));
        }

    }

    depth.dispatchEvent(new Event('input'));
	slidelimit.dispatchEvent(new Event('input'));

    $(".volume").click(function () {
        toggleOn();
    });


    if (navigator.requestMIDIAccess)
        navigator.requestMIDIAccess({
            sysex: false
        }).then(onMIDIInit, onMIDIReject);
    else
        alert("Your browser does not support MIDI. Please use Chrome or Opera, or the free WARBL iOS app.")

});


function updatePlatform()
{
    let newdisp =  (platform == "app" || platform == "app2") ? "none" : "block";
    //hide some stuff for app version
    document.getElementById("myTopnav").style.display = newdisp;
    document.getElementById("topLogo").style.display = newdisp;
    document.getElementById("importexport").style.display = newdisp;

    let newdispleg =  (platform == "app") ? "none" : "block";
    document.getElementById("midiSelector").style.display = newdispleg;
    document.getElementById("tinyDeviceLabel").style.display = newdispleg;        
}

//
// Common function to show the "WARBL not detected" message
//
function showWARBLNotDetected() {

    communicationMode = false; //make sure we know we're not connected
    document.getElementById("status").innerHTML = "WARBL not detected.";
    document.getElementById("status").style.color = "#F78339";
    document.getElementById("version").style.color = "#F78339";
    document.getElementById("connect").innerHTML = "Connect to WARBL";	//make sure the connect button shows the correct text
    document.getElementById("pressure").innerHTML = "";
    document.getElementById("connLabel").innerHTML = "BLE not connected";
    document.getElementById("connLabel").style.left = "95px";
    for (var m = 0; m < 9; m++) { //turn off toneholes
        document.getElementById("dot" + m).style.backgroundColor = "#333";
        if (m == 0) {
            document.getElementById("dot0").style.opacity = "0";
        }
    }


    // Disable the import preset button
    $('#importPreset').attr('disabled', 'disabled');

}

//
// Common function to show the "Unknown" version message
//
function showWARBLUnknown() {

    document.getElementById("version").innerHTML = "Unknown";
    document.getElementById("current").style.visibility = "hidden";
    document.getElementById("status").style.visibility = "visible";
    document.getElementById("connect").innerHTML = "Connect to WARBL";	//make sure the connect button shows the correct text


    // Disable the import preset button
    $('#importPreset').attr('disabled', 'disabled');

}

function updateMidiDeviceSelector() {

    if (midiAccess) {

        var selector = document.getElementById("midiDeviceSelector");
        selector.length = 0;


        var autoelement = document.createElement("option");
        autoelement.value = 0;
        autoelement.text = "Auto";
        selector.append(autoelement);

        var iter = midiAccess.outputs.values();
        var currSelectionExists = false;

        for (var o = iter.next(); !o.done; o = iter.next()) {

            if (o.value) {
                //console.log("Output " + o.value);
                var element = document.createElement("option");
                if (o.value.id) {
                    element.value = o.value.id;
                    if (o.value.id == selectedMidiOutId) {
                        currSelectionExists = true;
                    }
                }
                element.text = o.value.name;
                selector.append(element);
            }
        }

        if (WARBLout) {
            selector.value = WARBLout.id;
        }
        else if (currSelectionExists) {
            selector.value = selectedMidiOutId;
        }
        else {
            // set to auto
            selector.value = 0;
        }
   }
}


function midiDeviceSelected(value) {
    console.log("Midi device selected: " + value);

    selectedMidiOutId = value;
    startMidiConnect();

}

function enableMidiInputForOnly(portid)
{
    if (midiAccess) {
        var inputs = midiAccess.inputs.values();

        for (var input = inputs.next(); input && !input.done; input = inputs.next()) {

            if (input.value.id == portid) {
                input.value.onmidimessage = WARBL_Receive;
                //input.value.addEventListener('midimessage', WARBL_Receive, false);
            } else {
                input.value.onmidimessage = null;
                //input.value.removeEventListener('midimessage', WARBL_Receive, false);
            }
        }
    }
}

function enableMidiInputsForAll()
{
    if (midiAccess) {
        var inputs = midiAccess.inputs.values();

        for (var input = inputs.next(); input && !input.done; input = inputs.next()) {
            input.value.onmidimessage = WARBL_Receive;

            // for legacy app purposes
            // Use onstatechange to detect when any port is disconnected
            // the new app doesn't require this
            if (platform == "app") {
                input.value.onstatechange = function (event) {
                    var port = event.port;
                    console.log("MIDIInputPort onstatechange name:" + port.name + " connection:" + port.connection + " state:" + port.state);
                    if (port.state == "disconnected") {
                        // Assume the WARBL has been disconnected if any port state changes to "disconnected".
                        // since the legacy app doesn't know which port was being used for the WARBL
                        setToDisconnected();
                    }
                };
            }
        }
        console.log("Number midi inputs: " + midiAccess.inputs.size);
        return midiAccess.inputs.size > 0;
    }
    else {
        return false;
    }
}

// is called for connect or disconnect
function connect() {

    //console.log("connect function");

    if (communicationMode && version > 2.0) {
        //sendToAll(102, 99); //tell WARBL to exit communications mode if the "connect" button currently reads "Disconnect"	
        console.log("DISCONNECTING");

        if (version < 4.1){
            var cc = buildMessage(MIDI_CC_102, MIDI_CC_102_VALUE_99); //tell WARBL to exit communications mode if the "connect" button currently reads "Disconnect"
        }
        else {var cc = buildMessage(MIDI_CC_102, MIDI_CC_102_VALUE_104);}


        if (WARBLout && WARBLout != 1) { // just in case
            WARBLout.send(cc); //send CC message
        }

        setToDisconnected();

        enableMidiInputForOnly(0); // disable all midi input
    }
    else {
        // CONNECTING

        startMidiConnect();
    }

}

async function startMidiConnect()
{
    // ensure disconnected
    setToDisconnected();

    console.log("startMidiConnect");

    var outputId = document.getElementById("midiDeviceSelector").value;
    var isAutoMode = outputId == 0;

    enableMidiInputsForAll();

    var cc = buildMessage(MIDI_CC_102, MIDI_ENTER_COMM_MODE);

    // We send a command to specified (or all with AUTO mode) device until one responds

    //console.log("Sending enter comm mode to all manually with sleeps")
    midiConnecting = true;

    // for speed when there are many devices, try the ones with WARBL in the name first
    var iter = midiAccess.outputs.values();
    for (var o = iter.next(); !o.done; o = iter.next()) {
        if (!communicationMode) {

            if ((outputId == 0 || o.value.id == outputId) && o.value.name.includes("WARBL")) {
                console.log("Attempting connect to " + o.value.name + " id: " + o.value.id)
                midiScanningOut = o.value;
                o.value.send(cc); //send CC message to all ports
            }
            else {
                continue;
            }
        } else {
            // communication has been achieved, get out of this loop
            break;
        }
        await sleep(250); //This should be enough even for BLE
    }

    if (!communicationMode) {
        // now try all the rest ( I know this is an ugly copy/paste)
        var iter = midiAccess.outputs.values();
        for (var o = iter.next(); !o.done; o = iter.next()) {
            if (!communicationMode) {

                if ((outputId == 0 || o.value.id == outputId) && !o.value.name.includes("WARBL") ) {
                    console.log("Attempting connect to " + o.value.name + " id: " + o.value.id)
                    midiScanningOut = o.value;
                    o.value.send(cc); //send CC message to all ports
                }
                else {
                    continue;
                }
            } else {
                // communication has been achieved, get out of this loop
                break;
            }
            await sleep(250); //This should be enough even for BLE
        }
    }

    midiScanningOut = null;
    midiConnecting = false;

    // if we still don't have a connection, disable all midi inputs again
    if (!communicationMode) {
        console.log("Could not detect any connected WARBLs");
        enableMidiInputForOnly(0); // disable all midi input
        showWARBLNotDetected();
    }

}

//
// Callback when first requesting WebMIDI support
//

async function onMIDIInit(midi)  {

    midiAccess = midi;
    midi.onstatechange = midiOnStateChange;

    // Null the WARBL MIDI output port
    WARBLout = null;
    WARBLin = null;

    var foundMIDIInputDevice = enableMidiInputsForAll();

    updateMidiDeviceSelector();

    if (foundMIDIInputDevice) {
        // try a connection
        console.log("trying initial connect");
        startMidiConnect();
    }
    else {
        console.log("no inputs");

        showWARBLNotDetected();
    }
}


function onMIDIReject(err) {

    alert("The MIDI system failed to start. Please refresh the page.");

}

function setToDisconnected()
{
    showWARBLNotDetected();
    showWARBLUnknown();
    WARBLout = null;
    WARBLin = null;
    communicationMode = false;
    previousVersion = 0;
    document.getElementById("connect").innerHTML = "Connect to WARBL";	//make sure the connect button shows the correct text    
}

function midiOnStateChange(event) {

    console.log("midi state change");

    // make sure midi output port still exists, otherwise show disconnected
    
    if (WARBLout) {
        var iter = midiAccess.outputs.values();
        var foundit = false;
        for (var o = iter.next(); !o.done; o = iter.next()) {
            console.log("Searching out " + o.value.id + "  wout: " + WARBLout.id);
            if (o.value == WARBLout) {
                foundit = true;
                break;
            }
        }
        if (!foundit) {
            selectedMidiOutId = 0;
            setToDisconnected();
            enableMidiInputForOnly(0); // disable all midi input
        }
    }

    updateMidiDeviceSelector();
}

function setPing() {

    //change ping to 1 after 3 seconds, after which we'll show a disconnnect if the MIDI state changes.
    setTimeout(function () {

        ping = 1;

    }, 3000);
}


//
// Common message handler for sendToAll and sendToWARBL to build send message
//
function buildMessage(byte2, byte3) {



    if (!(byte2 == MIDI_CC_102 && byte3 == MIDI_START_CALIB) && !(byte2 == MIDI_CC_106 && byte3 == MIDI_CALIB_BELL_SENSOR)) {
        blink(1);
    } //blink once if we aren't doing autocalibration, which requires the LED to be on longer.

    if (byte2 == MIDI_CC_102) {
        if (byte3 == MIDI_SAVE_CALIB) { //send message to save sensor calibration
            blink(3);
            for (var i = 1; i < 10; i++) {
                document.getElementById("v" + (i)).innerHTML = "0";
            }
            for (var i = 0; i < 19; i++) {
                sensorValue[i] = 0;
            }
        }

        if (isEven(byte3) && byte3 <= MIDI_CALIB_MSGS_END) { //send sensor calibration values
            sensorValue[byte3 - 2]++;
            document.getElementById("v" + (byte3 >> 1)).innerHTML = sensorValue[byte3 - 2];
        }

        if (isOdd(byte3) && byte3 <= MIDI_CALIB_MSGS_END) {
            sensorValue[byte3 - 1]--;
            document.getElementById("v" + ((byte3 + 1) >> 1)).innerHTML = sensorValue[byte3 - 1];
            checkMax((byte3 + 1) >> 1);
        }
    }


    cc = [0xB6, byte2, byte3]; //prepare message



    return cc;
}




// Send to all connected MIDI devices
function sendToAll(byte2, byte3) {

    var cc = buildMessage(byte2, byte3);
    sendAllQueue.push(cc); //add message to queue
    if (!sendAllInterval) {
        sendAllInterval = setInterval(sendAllQueuedMessages, sendDelay); //start interval for sending queued messages
    }
}





// Send a command to only the WARBL output port
function sendToWARBL(byte2, byte3) {
    if (communicationMode) {
        
        // only used in legacy versions of the app
        if (platform == "app") {
            var cc = buildMessage(byte2, byte3);
            sendAllQueue.push(cc); //add message to queue
            if (!sendAllInterval) {
                sendAllInterval = setInterval(sendAllQueuedMessages, sendDelay); //start interval for sending queued messages
            }
        }
        else 
        {

            // Make sure we have a WARBL output port
            if (!WARBLout) {
                console.error("sendToWARBL: No MIDI port selected!")
                return;
            }

            var cc = buildMessage(byte2, byte3);
            if (MIDI_DEBUG) {
                console.log("sendToWARBL",byte2, byte3 );
            }
            sendWARBLoutQueue.push(cc); //add message to queue
            if (!sendWARBLInterval) {
                sendWARBLInterval = setInterval(sendQueuedWARBLoutMessages, sendDelay); //start interval for sending queued messages
            }

        }
    }
}




//send queued MIDI messages to WARBLout port at a regular internal to allow WARBL time to receive and process them all
function sendQueuedWARBLoutMessages() {
    if (sendWARBLoutQueue.length > 0) {
        //console.log("Sending to WARBL port");
        var cc = sendWARBLoutQueue.shift(); //shift next value from queue	
        WARBLout.send(cc); //send CC message
    }
    else {
        clearInterval(sendWARBLInterval);
        sendWARBLInterval = null;
    } //stop flushing queue if it's empty
}




//send queued MIDI messages to all ports at a regular internal to allow WARBL time to receive and process them all
function sendAllQueuedMessages() {
    if (sendAllQueue.length > 0) {
        //console.log("Sending to all ports");
        var cc = sendAllQueue.shift(); //shift next value from queue

        var iter = midiAccess.outputs.values();
        for (var o = iter.next(); !o.done; o = iter.next()) {
            o.value.send(cc); //send CC message to all ports
        }
    }
    else {
        clearInterval(sendAllInterval);
        sendAllInterval = null;
    } //stop flushing queue if it's empty		
}





function WARBL_Receive(event, source) {

    // only the app version will send this with a source parameter here 
    // (because for some reason the event.target doesnnt propogate on the iOS webkit side)
    //debugger;

    var data0 = event.data[0];
    var data1 = event.data[1];
    var data2 = event.data[2];

    //alert(WARBLout);

    // If we're exporting a preset, send the data to the exporter instead
    if (gExportProxy) {
        gExportProxy(data0, data1, data2);
        return;
    }

    //console.log("event target: " + event.target + " source: " + source);
    source = event.target != null ? event.target : source;

    //console.log("WARBL_Receive");
    //console.log(communicationMode);
    //console.log("WARBL_Receive target = "+event.target.name);
	
    // If we haven't established the WARBL output port and we get a received CC110 message on channel 7 (the first message that the WARBL sends back when connecting)
    // find the port by name by walking the output ports and matching the input port name
    if ((!WARBLout || !WARBLin) && ((data0 & 0x0F) == MIDI_CONFIG_TOOL_CHANNEL-1) && ((data0 & 0xf0) == 176) && (data1 == MIDI_CC_110)) {
        //alert(data0 & 0x0F);
        console.log("Platform: " + platform);
        if (platform == "app")  { // legacy app
            console.log("Legacy app connection");
            WARBLout = 1; //for legacy app version we don't worry about the device name or port, just that it's sending on channel 7.
            WARBLin = source;
        }
        else  {
            console.log("midiScanningOut: " + midiScanningOut + " id: " + midiScanningOut.id );
            if (!WARBLout && midiScanningOut) {
                WARBLout = midiScanningOut;
                selectedMidiOutId = WARBLout.id;
            }

            WARBLin = source;

            if (WARBLout && WARBLin) {
                console.log("Connected WARBL!")
                console.log(WARBLin);
                enableMidiInputForOnly(WARBLin.id);
                updateMidiDeviceSelector();
            }
        }
        

    }


    if (WARBLout != 1 && (!WARBLin || WARBLin != source)) {
        console.log("Ignoring midi message from unconnected device");
        return;
    }

    var e;
    var f;
    var g;

    if (isNaN(data2)) {
        f = " ";
    } else {
        f = data2;
    }

    if ((data0 & 0xf0) == 144) {
        e = "On";
    } //update the MIDI console
    if ((data0 & 0xf0) == 128) {
        e = "Off";
    }
    if ((data0 & 0xf0) == 176) {
        e = "CC";
    }
    if ((data0 & 0xf0) == 224) {
        e = "PB";
    }
    if ((data0 & 0xf0) == 192) {
        e = "PC";
    }
    if ((data0 & 0xf0) == 208) {
        e = "CP";
    }
    if ((data0 & 0xf0) == 160) {
        e = "KP";
    }


    //Display received PB messages
    if (e == "PB") {	
		var pitch = ((data1 + (data2 << 7)))-8192;	
        document.getElementById('receivedPB').innerHTML = pitch;
		var pitchcents = 100 * (pitch / 8192.0) * document.getElementById("midiBendRange").value;	
        document.getElementById('receivedPBCents').innerHTML = (pitchcents > 0 ? "+" : "") + pitchcents.toFixed(0); // jlc
        var barHeight = pitch / 81.92;
		var heightPixels = 65 - (barHeight/100*65) + "px";
		var markerTopPixels = 65 - (barHeight/100*65) + 90 + "px";
		document.getElementById("PBmarker").style.top = markerTopPixels;
		if(pitch < 0){
        	document.getElementById("PBlevel").style.height = -barHeight + "%";
			document.getElementById("topPBlevel").style.height = 0 + "%";
		}
		else if(pitch > 0){
			document.getElementById("PBlevel").style.height = 0 + "%";
			document.getElementById("topPBlevel").style.height = barHeight + "%";
			document.getElementById("topPBlevel").style.top = heightPixels;
		}
		else {
			document.getElementById("PBlevel").style.height = 0 + "%";
			document.getElementById("topPBlevel").style.height = 0 + "%";
		}

    }


    //If we receive a CC on the channel and CC# currently assigned in the IMU mapping panel, display the CC value
    var IMUchan = document.getElementById('IMUChannelInput').value;
    var IMUCC = document.getElementById('IMUCC').value;
    if (e == "CC" && (parseFloat(data0 & 0x0f) + 1 == IMUchan) && data1 == IMUCC) {
        document.getElementById('receivedCC').innerHTML = data2;
        var barWidth = data2 / 1.27;
        document.getElementById("CClevel").style.width = barWidth + "%";
    }
	
	
	//If we receive a message currently assigned in the pressure mapping panel, display the value
	if(mapSelection == 0){
    var IMUchan = document.getElementById('pressureChannel').value;
    var IMUCC = document.getElementById('pressureCC').value;
    if (e == "CC" && (parseFloat(data0 & 0x0f) + 1 == IMUchan) && data1 == IMUCC) {
        document.getElementById('receivedpressureCC').innerHTML = data2;
        var barWidth = data2 / 1.27;
        document.getElementById("CCpressurelevel").style.width = barWidth + "%";
	}
	}
	
	if(mapSelection == 1){
    if (e == "On") {
        document.getElementById('receivedpressureCC').innerHTML = data2;
        var barWidth = data2 / 1.27;
        document.getElementById("CCpressurelevel").style.width = barWidth + "%";
	}
	}
	
	if(mapSelection == 2){
		//console.log(2);
    if (e == "CP") {
        document.getElementById('receivedpressureCC').innerHTML = data1;
        var barWidth = data1 / 1.27;
        document.getElementById("CCpressurelevel").style.width = barWidth + "%";
	}
	}
	
	if(mapSelection == 3){
    if (e == "KP") {
        document.getElementById('receivedpressureCC').innerHTML = data2;
        var barWidth = data2 / 1.27;
        document.getElementById("CCpressurelevel").style.width = barWidth + "%";
	}
	}
	



    if (!(e == "CC" && (parseFloat(data0 & 0x0f) == MIDI_CONFIG_TOOL_CHANNEL-1))) { //as long as it's not a CC on channel 7, show in the MIDI console.
        consoleEntries++;
        if (consoleEntries < 301) {
            document.getElementById("console").innerHTML += (e + " " + ((data0 & 0x0f) + 1) + " " + data1 + " " + f);
            document.getElementById("console").innerHTML += "<br>";
        }
        else if (consoleEntries == 301) {
            document.getElementById("console").innerHTML += "max lines reached";
        }
        var elem2 = document.getElementById("console");
        elem2.scrollTop = elem2.scrollHeight;
    }

    // Mask off the lower nibble (MIDI channel, which we don't care about yet)
    switch (data0 & 0xf0) {
        case 0x90:
            if (data2 != 0) { // if velocity != 0, this is a note-on message
                noteOn(data1);
				notePlaying = data1; //keep track of most recent note so we know when to turn off the display.
                logKeys;
				document.getElementById("tinyConsole").value = noteNames[data1 - 1] + " " + data1;
                return;
            }
        // if velocity == 0, fall thru: it's a note-off.
        case 0x80:
            noteOff(data1);
            logKeys;
			if (notePlaying == data1) {document.getElementById("tinyConsole").value = "";}
            return;

        case 0xB0: //incoming CC from WARBL
            if (MIDI_DEBUG) {
                console.log("From WARBL", data1, data2);
            }
            if (parseFloat(data0 & 0x0f) ==  MIDI_CONFIG_TOOL_CHANNEL-1) { //if it's channel 7 it's from WARBL 

                //console.log("WARBL_Receive: "+data0+" "+data1+" "+data2);

                // Enable the import preset button
                $('#importPreset').removeAttr('disabled')

				/*
				if (communicationMode == false){ // If we think we're disconnected but receive a message from the WARBL, try to reconnect.
					connect();
				}
				*/

                if (data1 == MIDI_CC_115) { //hole covered info from WARBL

                    var byteA = data2;
                    for (var m = 0; m < 7; m++) {
                        if (bit_test(byteA, m) == 1) {
                            document.getElementById("dot" + m).style.backgroundColor = "blue";
                            if (m == 0) {
                                document.getElementById("dot0").style.opacity = "0.8";
                            }
                        } else {
                            document.getElementById("dot" + m).style.backgroundColor = "#333";
                            if (m == 0) {
                                document.getElementById("dot0").style.opacity = "0";
                            }
                        }
                    }
                }

                if (data1 == MIDI_CC_114) { //hole covered info from WARBL
                    for (var n = 0; n < 2; n++) {
                        var byteB = data2;
                        if (bit_test(byteB, n) == 1) {
                            document.getElementById("dot" + (7 + n)).style.backgroundColor = "blue";
                        } else {
                            if (n == 1) {
                                document.getElementById("dot" + (7 + n)).style.backgroundColor = "#181818";
                            }
                            if (n == 0) {
                                document.getElementById("dot" + (7 + n)).style.backgroundColor = "#333";
                            }
                        }
                    }
                } else if (data1 == MIDI_CC_102) { //parse based on received CC
                    if (data2 >= MIDI_MAX_CALIB_MSGS_START && data2 <= MIDI_MAX_CALIB_MSGS_END) {
                        document.getElementById("v" + (data2 - MIDI_MAX_CALIB_MSGS_START -1)).innerHTML = "MAX"; //set sensor value field to max if message is received from WARBL
                        checkMax((data2 - MIDI_MAX_CALIB_MSGS_START -1));
                    }

                    for (var i = 0; i < 3; i++) { // update the three selected fingering patterns if prompted by the tool.
                        if (data2 == MIDI_FINGERING_PATTERN_MODE_START + i) {
                            fingeringWrite = i;
                        }

                        if ((data2 >= MIDI_FINGERING_PATTERN_START && data2 <= MIDI_FINGERING_PATTERN_END)) {

                            if (fingeringWrite == i) {
                                document.getElementById("fingeringSelect" + i).value = data2 - MIDI_FINGERING_PATTERN_START;
                            }
                            updateCells(); //update any dependant fields	
                        }
						
						if((data2 >= MIDI_CUST_FINGERING_PATTERN_START && data2 <= MIDI_CUST_FINGERING_PATTERN_END) && version > 3.9){
							if (fingeringWrite == i) {
                                document.getElementById("fingeringSelect" + i).value = data2 - MIDI_FINGERING_PATTERN_START;
                            }
                            updateCells(); //update any dependant fields	
                        }

                    }



                    if (data2 == MIDI_CURRENT_MODE_START) {
                        document.getElementById("fingering0").checked = true;
                        instrument = 0;
                        document.getElementById("instrument0").style.display = "block";
                        document.getElementById("instrument1").style.display = "none";
                        document.getElementById("instrument2").style.display = "none";
                        document.getElementById("key0").style.display = "block";
                        document.getElementById("key1").style.display = "none";
                        document.getElementById("key2").style.display = "none";
                        advancedOkay(); //turn off the advanced tab	
                        pressureOkay();
                        updateCells();
                        advancedOkayPB();
                        okayCCmap();
                        okayIMUmap();
						okayPitchRegistermap();
                        okayOverride();
                        backIMU();
                        backPressure();
                        handleDefault();
                        customFingeringOkay();
                    }
                    if (data2 == MIDI_CURRENT_MODE_START +1) {
                        document.getElementById("fingering1").checked = true;
                        instrument = 1;
                        document.getElementById("instrument0").style.display = "none";
                        document.getElementById("instrument1").style.display = "block";
                        document.getElementById("instrument2").style.display = "none";
                        document.getElementById("key0").style.display = "none";
                        document.getElementById("key1").style.display = "block";
                        document.getElementById("key2").style.display = "none";
                        advancedOkay(); //turn off the advanced tab	
                        pressureOkay();
                        updateCells();
                        advancedOkayPB();
                        okayCCmap();
                        okayIMUmap();
						okayPitchRegistermap();
                        okayOverride();
                        backIMU();
                        backPressure();
                        handleDefault();
                        customFingeringOkay();
                    }
                    if (data2 == MIDI_CURRENT_MODE_START +2) {
                        document.getElementById("fingering2").checked = true;
                        instrument = 2;
                        document.getElementById("instrument0").style.display = "none";
                        document.getElementById("instrument1").style.display = "none";
                        document.getElementById("instrument2").style.display = "block";
                        document.getElementById("key0").style.display = "none";
                        document.getElementById("key1").style.display = "none";
                        document.getElementById("key2").style.display = "block";
                        advancedOkay(); //turn off the advanced tab	
                        pressureOkay();
                        updateCells();
                        advancedOkayPB();
                        okayCCmap();
                        okayIMUmap();
						okayPitchRegistermap();
                        okayOverride();
                        backIMU();
                        backPressure();
                        handleDefault();
                        customFingeringOkay();
                    }

                    // if (data2 == MIDI_DEFAULT_MODE_START) { //receive and handle default instrument settings
                    //     defaultInstr = 0;
                    //     handleDefault();
                    // }
                    // if (data2 == MIDI_DEFAULT_MODE_START +1) {
                    //     defaultInstr = 1;
                    //     handleDefault();
                    // }
                    // if (data2 == MIDI_DEFAULT_MODE_START+2) {
                    //     defaultInstr = 2;
                    //     handleDefault();
                    // }
                    for (var i = 0; i < 3; i++)  { //receive and handle default instrument settings
                        if (data2 == MIDI_DEFAULT_MODE_START +i) {
                            defaultInstr = i;
                            handleDefault();
                        }
                    }

                    // if (data2 == MIDI_PB_MODE_START) {
                    //     document.getElementById("pitchbendradio0").checked = true;
                    //     updateCustom();
                    //     updateCustom();
                    // }
                    // if (data2 == 71) {
                    //     document.getElementById("pitchbendradio1").checked = true;
                    //     updateCustom();
                    //     updateCustom();
                    // }
                    // if (data2 == 72) {
                    //     document.getElementById("pitchbendradio2").checked = true;
                    //     updateCustom();
                    //     updateCustom();
                    // }
                    // if (data2 == 73) {
                    //     document.getElementById("pitchbendradio3").checked = true;
                    //     updateCustom();
                    //     updateCustom();
                    // }
                    for (var i = 0; i < 4; i++)  { //receive and handle Pitchbend mode
                        if (data2 == MIDI_PB_MODE_START +i) {
                            document.getElementById("pitchbendradio" + i.toString()).checked = true;
                            updateCustom();
                            updateCustom();    
                        }
                    }


                    // if (data2 == 80) {
                    //     document.getElementById("sensorradio0").checked = true;
                    // }
                    // if (data2 == 81) {
                    //     document.getElementById("sensorradio1").checked = true;
                    // }
                    // if (data2 == 82) {
                    //     document.getElementById("sensorradio2").checked = true;
                    // }
                    // if (data2 == 83) {
                    //     document.getElementById("sensorradio3").checked = true;
                    // }
                    for (var i = 0; i < 4; i++)  { //receive and handle Breath mode
                        if (data2 == MIDI_BREATH_MODE_START +i) {
                            document.getElementById("sensorradio" + i.toString()).checked = true;
                            if (i == 1) {
                                updateExpressionSliderEnableState();
                            }
                        }
                    }


                    if (data2 == MIDI_CC_102_VALUE_121) { //bell sensor connected	
                        document.getElementById("bellSensor").style.opacity = 1;
                        document.getElementById("1").disabled = false;
                        document.getElementById("2").disabled = false;
                        document.getElementById("v1").classList.add("sensorValueEnabled");

                    }
                    if (data2 == MIDI_CC_102_VALUE_120) { //bell sensor disconnected	
                        document.getElementById("bellSensor").style.opacity = 0.1;
                        document.getElementById("1").disabled = true;
                        document.getElementById("2").disabled = true;
                        document.getElementById("v1").classList.remove("sensorValueEnabled");
                    }


                    for (var i = 0; i < numberOfGestures; i++) { //update button configuration	   
                        if (data2 == MIDI_GESTURE_START + i) {
                            buttonRowWrite = i;
							//console.log(buttonRowWrite);
                        }
                    }

                    for (var j = 0; j < numberOfGestures; j++) { //update button configuration	
                        if (buttonRowWrite == j) {

                            if (data2 == 119) { //special case for initiating autocalibration - NOT in WARBL2?
                                document.getElementById("row" + (buttonRowWrite)).value = 19;
                            }


                            for (var k = 0; k < 12; k++) {
                                if (data2 == (100 + k) && version < 4.0) {
                                    document.getElementById("row" + (buttonRowWrite)).value = k;
                                }

                                if (k < 7 && data2 == MIDI_ACTION_MIDI_START + k) {
                                    document.getElementById("MIDIrow" + (buttonRowWrite)).value = k;
                                    updateCells();
                                }
                            }
                        }
                    } //update any dependant fields}						


                    for (var l = 0; l < 3; l++) { //update momentary switches
                        if (buttonRowWrite == l) {
                            if (data2 == MIDI_MOMENTARY_OFF) {
                                document.getElementById("checkbox" + l).checked = false;
                            }
                            if (data2 == MIDI_MOMENTARY_ON) {
                                document.getElementById("checkbox" + l).checked = true;
                            }

                        }
                    }

                } else if (data1 == MIDI_CC_103) {
                    document.getElementById("senseDistance").value = 0 - data2;
                } //set sensedistance  
                else if (data1 == MIDI_CC_117) {
                    document.getElementById("depth").value = data2 + 1;
                    var output = document.getElementById("demo14");
                    depth.dispatchEvent(new Event('input'));
                    output.innerHTML = data2 + 1;
                } //set vibrato depth  
                else if (data1 == MIDI_CC_104) {
                    jumpFactorWrite = data2;
                } // so we know which pressure setting is going to be received.
                else if (data1 == MIDI_CC_109 && data2 != MIDI_CUSTOM_CHARTS_RCVD) {
                    jumpFactorWrite = data2 + MIDI_CC_109_OFFSET;
                } // so we know which WARBL2 IMU setting is going to be received.
                else if (data1 == MIDI_CC_109 && data2 == MIDI_CUSTOM_CHARTS_RCVD) { //Successful WARBL2 Custom chart receipt
                    document.getElementById("sending").innerHTML = "Success!";
                    document.getElementById("WARBL2CustomSuccessOkay").style.display = "block";
                    document.getElementById('WARBL2customFingeringFill').value = '10';
                    document.getElementById("WARBL2CustomTextArea").value = '';
                }
                else if (data1 == MIDI_CC_105 && jumpFactorWrite <= MIDI_PRESS_SELECT_VARS_END) {
                    document.getElementById("jumpFactor" + jumpFactorWrite).value = data2;
                    if (jumpFactorWrite == MIDI_CC_104_VALUE_10) {
                        document.getElementById("jumpFactor10b").value = data2; //special case for hysteresis			
                    }
                    for (var i = 1; i <= MIDI_PRESS_SELECT_VARS_END; i++) {
                        var k = document.getElementById("jumpFactor" + (i));
                        k.dispatchEvent(new Event('input'));
                        if (i == MIDI_CC_104_VALUE_10) {
                            var k = document.getElementById("jumpFactor10b"); //special case for key delay slider
                            k.dispatchEvent(new Event('input'));
                        }
                    }
                }

                if (data1 == MIDI_CC_105) {

                    if (jumpFactorWrite == MIDI_ED_VARS_START) {
                        document.getElementById("checkbox6").checked = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +1) {
                        document.getElementById("expressionDepth").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +2) {
                        document.getElementById("checkbox7").checked = data2;
                        updateCustom();
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +3) {
                        curve[0] = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +4) {
                        document.getElementById("pressureChannel").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +5) {
                        document.getElementById("pressureCC").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +6) {
                        inputSliderMin[0] = data2;
                        slider.noUiSlider.set([data2, null]);
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +7) {
                        inputSliderMax[0] = data2;
                        slider.noUiSlider.set([null, data2]);
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +8) {
                        outputSliderMin[0] = data2;
                        slider2.noUiSlider.set([data2, null]);
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +9) {
                        outputSliderMax[0] = data2;
                        slider2.noUiSlider.set([null, data2]);
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +10) {
                        document.getElementById("dronesOnCommand").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +11) {
                        document.getElementById("dronesOnChannel").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +12) {
                        document.getElementById("dronesOnByte2").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +13) {
                        document.getElementById("dronesOnByte3").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +14) {
                        document.getElementById("dronesOffCommand").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +15) {
                        document.getElementById("dronesOffChannel").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +16) {
                        document.getElementById("dronesOffByte2").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +17) {
                        document.getElementById("dronesOffByte3").value = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +18 && data2 == 0) {
                        document.getElementById("dronesRadio0").checked = true;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +18 && data2 == 1) {
                        document.getElementById("dronesRadio1").checked = true;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +18 && data2 == 2) {
                        document.getElementById("dronesRadio2").checked = true;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +18 && data2 == 3) {
                        document.getElementById("dronesRadio3").checked = true;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +19) {
                        lsb = data2;
                    } else if (jumpFactorWrite == MIDI_ED_VARS_START +20) {
                        var x = parseInt((data2 << 7) | lsb); //receive pressure between 100 and 900
                        x = (x - 100) * 24 / 900; //convert to inches of water. 100 is the approximate minimum sensor value.
                        var p = x.toFixed(1); //round to 1 decimal
                        p = Math.min(Math.max(p, 0), 24); //constrain
                        document.getElementById("dronesPressureInput").value = p;
                    } else if (jumpFactorWrite == MIDI_LEARNED_PRESS_LSB) {
                        lsb = data2;

                    } else if (jumpFactorWrite == MIDI_LEARNED_PRESS_MSB) {
                        var x = parseInt((data2 << 7) | lsb); //receive pressure between 100 and 900
                        x = (x - 100) * 24 / 900; //convert to inches of water.  100 is the approximate minimum sensor value.
                        var p = x.toFixed(1); //round to 1 decimal
                        p = Math.min(Math.max(p, 0), 24); //constrain
                        document.getElementById("octavePressureInput").value = p;
                    } else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +3) {
                        document.getElementById("checkbox9").checked = data2;
                    } //invert											
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +4) {
                        document.getElementById("checkbox5").checked = data2; //custom
                        updateCustom();
                    } else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +2) {
                        document.getElementById("checkbox3").checked = data2;
                    } //secret										
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START) {
                        updateSelected();
                        document.getElementById("checkbox4").checked = data2;
                        if (data2 == 0) {
                            document.getElementById("bagBreath0").checked = true;
                        } else { document.getElementById("bagBreath1").checked = true; }
                    } //vented							 			
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +1) {
                        document.getElementById("checkbox8").checked = data2;
                    } //bagless						 	
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +5) {
                        document.getElementById("checkbox10").checked = data2;
                    } //velocity	
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +6) {
                        document.getElementById("checkbox11").checked = (data2 & 0x1);
                        document.getElementById("checkbox13").checked = (data2 & 0x2);
                    } //aftertouch	
                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +7) {
                        document.getElementById("checkbox12").checked = data2;
                    } //force max velocity

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +8) {
                        document.getElementById("checkbox14").checked = data2;
                    } //immediate pitchbend

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +9) {
                        document.getElementById("checkbox15").checked = data2;
                    } //legato

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +10) {
                        document.getElementById("checkbox16").checked = data2;
                        document.getElementById("overrideExprCheck").checked = data2;
                        
                        updateExpressionSliderEnableState();
                        
                    } //override expression pressure range

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +11) {
                        document.getElementById("checkbox17").checked = data2;
                    } //both thumb and overblow

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +12) {
                        document.getElementById("checkbox18").checked = data2;
                        updateCustom();
                    } //R4 flattens

                    else if (jumpFactorWrite == MIDI_SWITCHES_VARS_START +13) {
                        document.getElementById("cbDoubleClick").checked = data2;
						updateDoubleClick();
                    } //BUTTON_DOUBLE_CLICK
                    

                    else if (jumpFactorWrite == MIDI_BEND_RANGE) {
                        document.getElementById("midiBendRange").value = data2;
                    }
                    else if (jumpFactorWrite == MIDI_MIDI_CHANNEL) {
                        document.getElementById("noteChannel").value = data2;
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START) {
                        inputSliderMin[1] = data2;
                        slider.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +1) {
                        inputSliderMax[1] = data2;
                        slider.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +2) {
                        outputSliderMin[1] = data2;
                        slider2.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +3) {
                        outputSliderMax[1] = data2;
                        slider2.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +4) {
                        inputSliderMin[2] = data2;
                        slider.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +5) {
                        inputSliderMax[2] = data2;
                        slider.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +6) {
                        outputSliderMin[2] = data2;
                        slider2.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +7) {
                        outputSliderMax[2] = data2;
                        slider2.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +8) {
                        inputSliderMin[3] = data2;
                        slider.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +9) {
                        inputSliderMax[3] = data2;
                        slider.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +10) {
                        outputSliderMin[3] = data2;
                        slider2.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +11) {
                        outputSliderMax[3] = data2;
                        slider2.noUiSlider.set([null, data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +12) {
                        curve[1] = data2;
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +13) {
                        curve[2] = data2;
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +14) {
                        curve[3] = data2;
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +15) {
                        // pitch expression min (low min in new model)
                        exprlowslider.noUiSlider.set([data2, null]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +16) {
                        // pitch expression max (high max in new model)
                        if (version < 4.3) {
                            exprlowslider.noUiSlider.set([null, data2]);
                        } else {
                            exprhighslider.noUiSlider.set([null, data2]);                            
                        }
                    }					
                    else if (version < 4.0 && jumpFactorWrite >= MIDI_CC_104_VALUE_87 && jumpFactorWrite <= MIDI_ED_VARS2_END) { //custom fingering chart inputs -WARBL1
                        document.getElementById("fingeringInput" + (jumpFactorWrite - 86)).value = data2;
                    }			
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +17) {
						// slideLimit
						document.getElementById("slidelimit").value = data2 ;
	                    var output = document.getElementById("demo18");
	                    depth.dispatchEvent(new Event('input'));
	                    output.innerHTML = data2;
					}
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +18) {
                        // pitch expression  low max
                        if (version >= 4.3) {                            
                            exprlowslider.noUiSlider.set([null, data2]);
                        }
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +19) {
                        // pitch expression high min
                        exprhighslider.noUiSlider.set([data2, null]);
                    }					
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +20) {
                        // low bend
                        exprlowbendslider.noUiSlider.set([data2]);
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +21) {
                        // high bend
                        exprhighbendslider.noUiSlider.set([data2]);
                    }					
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +22) {
                        // fixed center pressure
                        document.getElementById("exprfixedcenterslider").value = data2;
                        updateExpressionCenterPressureLabel();
                    }
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +23) {
                        // expr max clamp
                        document.getElementById("clampExprCheck").checked = data2;                        
                    }					
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +24) {
                        // expr low curve
                        document.getElementById("exprcurvelowslider").value = data2;
                        updateExpressionCurveLabels();
                    }					
                    else if (jumpFactorWrite == MIDI_ED_VARS2_START +25) {
                        // expr low curve
                        document.getElementById("exprcurvehighslider").value = data2;
                        updateExpressionCurveLabels();
                    }					


                    else if (jumpFactorWrite >=  MIDI_CC_109_OFFSET) { //receiving WARBL2 IMU settings

                        if (jumpFactorWrite == MIDI_CC_109_OFFSET) {
                            document.getElementById("checkbox20").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +1) {
                            document.getElementById("checkbox21").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +2) {
                            document.getElementById("checkbox22").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +3) {
                            center[1] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +4) {
                            center[3] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +5) { //input (-90 to 90)
                            inputSliderMin[4] = (data2 * 5) - 90;
							//console.log("roll in min= "); 
							//console.log(data2); 
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +6) { //input (-90 to 90)
                            inputSliderMax[4] = (data2 * 5) - 90;
							//console.log("roll in max= "); 
							//console.log(data2); 
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +7) {
                            outputSliderMin[4] = data2;
							//console.log("roll out min= "); 
							//console.log(data2); 
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +8) {
                            outputSliderMax[4] = data2;
							//console.log("roll out max = "); 
							//console.log(data2);
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +9) { //input (-90 to 90)
                            inputSliderMin[5] = (data2 * 5) - 90;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +10) { //input (-90 to 90)
                            inputSliderMax[5] = (data2 * 5) - 90;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +11) {
                            outputSliderMin[5] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +12) {
                            outputSliderMax[5] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +13) { //Yaw input min (0-180)
                            inputSliderMin[6] = (data2 * 5) - 90;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +14) { //Yaw input max (0-180)
                            inputSliderMax[6] = (data2 * 5) - 90;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +15) {
                            outputSliderMin[6] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +16) {
                            outputSliderMax[6] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +17) {
                            IMUchannel[0] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +18) {
                            IMUchannel[1] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +19) {
                            IMUchannel[2] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +20) {
                            IMUnumber[0] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +21) {
                            IMUnumber[1] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +22) {
                            IMUnumber[2] = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +23) {
                            document.getElementById("checkbox25").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +24) {
                            document.getElementById("checkbox24").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +25) {
                            document.getElementById("autoCenterYawInterval").value = data2;
                            var output = document.getElementById("demo16");
                            autoCenterYawInterval.dispatchEvent(new Event('input'));
                            output.innerHTML = data2 / 4;
                        }
						else if (jumpFactorWrite == MIDI_CC_109_OFFSET +26) {
                            document.getElementById("checkbox27").checked = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +27) {
                            document.getElementById("shakeDepth").value = data2;
                            var output = document.getElementById("demo15");
                            shakeDepth.dispatchEvent(new Event('input'));
                            output.innerHTML = data2;
                        }		
						else if (jumpFactorWrite == MIDI_CC_109_OFFSET +28) { //input (-90 to 90)
							  slider6.noUiSlider.set([(data2 * 5) - 90, null]);
                        }
						else if (jumpFactorWrite == MIDI_CC_109_OFFSET +29) { //input (-90 to 90)
							  slider6.noUiSlider.set([null, (data2 * 5) - 90]);
                        }
						else if (jumpFactorWrite == MIDI_CC_109_OFFSET +30) {
                            document.getElementById("pitchRegisterNumber").value = data2;
                            var output = document.getElementById("demo17");
                            pitchRegisterNumber.dispatchEvent(new Event('input'));
                            output.innerHTML = data2;
                        }
                        else if (jumpFactorWrite == MIDI_CC_109_OFFSET +31) {
                            document.getElementById("shakeVibModeCommand").value = data2;
                        }
						
						//End of WARBL2 IMU settings
						
						else if (jumpFactorWrite == MIDI_CC_109_OFFSET + MIDI_CC_109_VALUE_127) { // Receive button/gesture action
						var gesture = document.getElementById("gestureLabel" + (data2));
						gesture.style.color = "#f7c839";
						gesture.style.scale="1.1"					
						setTimeout(function() {contract(gesture);}, 150);						
						}
						
                    } //End of WARBL2 IMU settings

                } //end of CC105



                /* MrMep: Should this be 33 instead of 23? */
                // else if (data1 == MIDI_CC_109 && data2 < 23) { //Indicates WARBL2 IMU setting will be received on CC105
                else if (data1 == MIDI_CC_109 && data2 <= MIDI_IMU_SETTINGS_END) { //Indicates WARBL2 IMU setting will be received on CC105
                    jumpFactorWrite = data2 + MIDI_CC_109_OFFSET;
                }


                else if (data1 == MIDI_CC_110) { //receiving firmware version from WARBL

                    version = data2;

                    if (previousVersion != 0 && version != previousVersion) { //If the version has changed because a different WARBL has connected
                        document.location.reload(false); //refresh the page without reloading to revert the settings
                    }

                    communicationMode = true; //moved above
                    previousVersion = version;
					

                    if ((version >= 40 && version >= currentVersion) || (version < 40 && version >= currentVersionOriginal)) { //display the appropriate messages
                        document.getElementById("current").innerHTML = "Your firmware is up to date.";
                        document.getElementById("current").style.left = "710px";
                        document.getElementById("current").style.visibility = "visible";
                        document.getElementById("status").style.visibility = "hidden";
                        document.getElementById("current").style.color = "#f7c839";
                    }
                    else {
                        document.getElementById("current").innerHTML = "There is a firmware update available.";
                        document.getElementById("current").style.left = "690px";
                        document.getElementById("current").style.visibility = "visible";
                        document.getElementById("status").style.visibility = "hidden";
                        document.getElementById("current").style.color = "#F78339";
                    }


                    version = version / 10;
                    var n = version.toFixed(1)
                    document.getElementById("version").innerHTML = n;
                    document.getElementById("version").style.color = "#f7c839";


                    //show that the WARBL is connected when we receive the firmware version.
                    //console.log("connect");
                    document.getElementById("status").innerHTML = "WARBL Connected.";
                    document.getElementById("status").style.color = "#f7c839";
                    // communicationMode = true; //moved above
                    if (version > 2.0) {
                        document.getElementById("connect").innerHTML = "Disconnect";
                    }







                    //Lots of UI changes based on the current firmware version of the connected WARBL


                    //add new items that should only be visible with newer software versions and didable ones that are for newer version than the current one.
					
					
					if (version < 4.2) {
					
						document.getElementById("switchDoubleClick").style.display = "none";
						document.getElementById("doubleClickLabel").style.display = "none";
						
					}
					


                    if (version < 3.9) {

						numberOfGestures = 8;
						for (let element of document.getElementsByClassName("WARBL2Elements")){element.style.display="none";}
                        document.getElementById("box9").style.display = "none";
                        document.getElementById("expressionPressureBox").style.display = "block";
						document.getElementById("demo18").style.display = "none";
						document.getElementById("slideLimitContainer").style.display = "none";
						document.getElementById("slidelimitlabel").style.display = "none";
						document.getElementById("advancedPBsliders").style.top = "50px";
                        document.getElementById("backPressureButton").style.display = "none";
                        document.getElementById("fingeringSelect0").options[22].disabled = true;
                        document.getElementById("fingeringSelect0").options[23].disabled = true;
                        document.getElementById("fingeringSelect0").options[24].disabled = true;
                        document.getElementById("fingeringSelect0").options[25].disabled = true;
                        document.getElementById("fingeringSelect1").options[22].disabled = true;
                        document.getElementById("fingeringSelect1").options[23].disabled = true;
                        document.getElementById("fingeringSelect1").options[24].disabled = true;
                        document.getElementById("fingeringSelect1").options[25].disabled = true;
                        document.getElementById("fingeringSelect2").options[22].disabled = true;
                        document.getElementById("fingeringSelect2").options[23].disabled = true;
                        document.getElementById("fingeringSelect2").options[24].disabled = true;
                        document.getElementById("fingeringSelect2").options[25].disabled = true;	
						
						document.getElementById("keyLabel").innerHTML = "Key";
						document.getElementById("keyLabel").style.left = "305px";
	
						for (i = 0; i < 37; ++i) {	
							document.getElementsByName('keySelect')[0].options[i].innerHTML = noteNamesforKeySelect[i];
							document.getElementsByName('keySelect')[1].options[i].innerHTML = noteNamesforKeySelect[i];
							document.getElementsByName('keySelect')[2].options[i].innerHTML = noteNamesforKeySelect[i];
						}

                        for (i = 0; i < document.getElementById("fingeringSelect0").length; ++i) { //add the "Custom" option back in
                            if (document.getElementById("fingeringSelect0").options[i].value == "19") {
                                var a = 1;
                            }
                        }

                        if (a != 1) {
                            var x = document.getElementById("fingeringSelect0");
                            var option = document.createElement("option");
                            option.text = "Custom (original WARBL)";
                            option.value = 19;
                            x.add(option);

                            var y = document.getElementById("fingeringSelect1");
                            var option = document.createElement("option");
                            option.text = "Custom (original WARBL)";
                            option.value = 19;
                            y.add(option);

                            var z = document.getElementById("fingeringSelect2");
                            var option = document.createElement("option");
                            option.text = "Custom (original WARBL)";
                            option.value = 19;
                            z.add(option);
                        }

                        for (i = 0; i < 8; ++i) {

                            document.getElementById("row" + i).options[13].disabled = true; //disable Power down, recenter yaw, batt level options
                            document.getElementById("row" + i).options[14].disabled = true;
							document.getElementById("row" + i).options[15].disabled = true;

                            var opt = document.getElementById("autoCal" + i); //Change autocalibrate option value to 19 to work with original WARBL firmware
                            opt.value = '19';
                        }


                        document.getElementById("WARBL2Settings").style.display = "none"; //hide the WARBL2 settings			
                        document.getElementById("bellSensor").style.display = "block"; //show the bell sensor drawing
                        document.getElementById("dot0").style.left = "935px"; //move bell sensor stuff over
                        document.getElementById("1").style.left = "900px";
                        document.getElementById("2").style.left = "900px";
                        document.getElementById("v1").style.left = "900px";

						

                    }

                    else { //WARBL2

                        for (i = 0; i < 8; ++i) {

                            document.getElementById("row" + i).options[13].disabled = false; //disable Power down and recenter yaw options
                            document.getElementById("row" + i).options[14].disabled = false;
							document.getElementById("row" + i).options[15].disabled = false;
                            var opt = document.getElementById("autoCal" + i); //Change autocalibrate option value to 19 to work with original WARBL firmware
                            opt.value = '12';
                        }


                        document.getElementById("WARBL2Settings").style.display = "block"; //hide the WARBL2 settings			
                        document.getElementById("bellSensor").style.display = "none"; //show the bell sensor drawing
                        document.getElementById("dot0").style.left = "870px"; //move bell sensor stuff over
                        document.getElementById("1").style.left = "870px";
                        document.getElementById("2").style.left = "870px";
                        document.getElementById("v1").style.left = "870px";



                    }



                    if (version < 2.2) {

                        //disable medieval pipes and bansuri fingering option
                        document.getElementById("fingeringSelect0").options[20].disabled = true;
                        document.getElementById("fingeringSelect1").options[20].disabled = true;
                        document.getElementById("fingeringSelect2").options[20].disabled = true;
                        document.getElementById("fingeringSelect0").options[21].disabled = true;
                        document.getElementById("fingeringSelect1").options[21].disabled = true;
                        document.getElementById("fingeringSelect2").options[21].disabled = true;
                    }

                    if (version < 2.1) {
                        document.getElementById("jumpfactor10Container").style.visibility = "visible";
                        document.getElementById("jumpfactor4Container").style.visibility = "visible";
                        document.getElementById("jumpValue4").style.visibility = "visible";
                        document.getElementById("jumpValue10").style.visibility = "visible";
                        document.getElementById("jumpfactor10bContainer").style.visibility = "hidden";
                        document.getElementById("jumpLabel8").style.visibility = "hidden";
                        document.getElementById("jumpLabel5").style.visibility = "visible";
                        document.getElementById("demo10b").style.visibility = "hidden";
                        document.getElementById("jumpLabel4").innerHTML = "Jump";
                        document.getElementById("jumpfactor5Container").style.top = "240px";
                        document.getElementById("jumpfactor6Container").style.top = "275px";
                        document.getElementById("jumpfactor11Container").style.top = "240px";
                        document.getElementById("jumpfactor12Container").style.top = "275px";
                        document.getElementById("jumplabel6").style.top = "242px";
                        document.getElementById("jumplabel7").style.top = "277px";
                        document.getElementById("jumpValue5").style.top = "225px";
                        document.getElementById("jumpValue6").style.top = "260px";
                        document.getElementById("jumpValue11").style.top = "225px";
                        document.getElementById("jumpValue12").style.top = "260px";
                        document.getElementById("jumpFactor3").min = "1";
                        document.getElementById("jumpFactor3").max = "175";
                        document.getElementById("jumpFactor9").min = "1";
                        document.getElementById("jumpFactor9").max = "175";
                        document.getElementById("jumpFactor5").min = "1";
                        document.getElementById("jumpFactor6").min = "1";
                        document.getElementById("jumpFactor11").min = "1";
                        document.getElementById("jumpFactor12").min = "1";
                        document.getElementById("breathSlider").classList.remove('slider2');
                        document.getElementById("breathSlider").classList.add('slider');
                        document.getElementById("pressureGroup").style.top = "0px";
                        document.getElementById("ventedLabel").style.visibility = "visible";
                        document.getElementById("switch4").style.visibility = "visible";
                        document.getElementById("radioGroup10").style.visibility = "hidden";
                        document.getElementById("jumplabel10").innerHTML = "Closed";
                        document.getElementById("jumplabel11").innerHTML = "Vented";
                        document.getElementById("jumplabel10").style.left = "170px";
                        document.getElementById("jumplabel11").style.left = "340px";
                        document.getElementById("advancedSliders").style.top = "-125px";
                        document.getElementById("selectedSettings").style.visibility = "hidden";




                        //disable bombarde and baroque flute fingering option
                        document.getElementById("fingeringSelect0").options[19].disabled = true;
                        document.getElementById("fingeringSelect1").options[19].disabled = true;
                        document.getElementById("fingeringSelect2").options[19].disabled = true;
                        document.getElementById("fingeringSelect0").options[20].disabled = true;
                        document.getElementById("fingeringSelect1").options[20].disabled = true;
                        document.getElementById("fingeringSelect2").options[20].disabled = true;

                    }



                    if (version < 2.0) {
                        document.getElementById("overrideExpression").disabled = true;
                        document.getElementById("polyMapButton").disabled = true;
                        document.getElementById("velocityMapButton").disabled = true;
                        document.getElementById("aftertouchMapButton").disabled = true;
                        document.getElementById("noteChannel").disabled = true;
                        document.getElementById("checkbox13").disabled = true;
                        document.getElementById("switch13").style.cursor = "default";
                        document.getElementById("advancedPBbutton").disabled = true;
                        document.getElementById("pitchbendradio3").disabled = true;
                        document.getElementById("configureCustomFingering").disabled = true;
                    }



                    if (version < 1.5) {
                        document.getElementById("fingeringSelect0").options[2].disabled = true;
                        document.getElementById("fingeringSelect1").options[2].disabled = true;
                        document.getElementById("fingeringSelect2").options[2].disabled = true;
                        document.getElementById("checkbox10").disabled = true;
                        for (var i = 0; i < 8; i++) {
                            document.getElementById("row" + i).options[7].disabled = true;
                            document.getElementById("row" + i).options[8].disabled = true;
                        }
                    }

                    if (version < 1.6) {
                        for (var i = 0; i < 8; i++) {
                            document.getElementById("row" + i).options[12].disabled = true;
                        }
                        document.getElementById("fingeringSelect0").options[6].disabled = true;
                        document.getElementById("fingeringSelect1").options[6].disabled = true;
                        document.getElementById("fingeringSelect2").options[6].disabled = true;
                        document.getElementById("fingeringSelect0").options[11].disabled = true;
                        document.getElementById("fingeringSelect1").options[11].disabled = true;
                        document.getElementById("fingeringSelect2").options[11].disabled = true;
                    }


                    if (version < 1.7) {
                        document.getElementById("checkbox11").disabled = true;
                        document.getElementById("checkbox12").disabled = true;

                        document.getElementById("fingeringSelect0").options[12].disabled = true;
                        document.getElementById("fingeringSelect1").options[12].disabled = true;
                        document.getElementById("fingeringSelect2").options[12].disabled = true;

                        document.getElementById("fingeringSelect0").options[13].disabled = true;
                        document.getElementById("fingeringSelect1").options[13].disabled = true;
                        document.getElementById("fingeringSelect2").options[13].disabled = true;

                        document.getElementById("fingeringSelect0").options[14].disabled = true;
                        document.getElementById("fingeringSelect1").options[14].disabled = true;
                        document.getElementById("fingeringSelect2").options[14].disabled = true;

                        document.getElementById("fingeringSelect0").options[15].disabled = true;
                        document.getElementById("fingeringSelect1").options[15].disabled = true;
                        document.getElementById("fingeringSelect2").options[15].disabled = true;
                    }


                    if (version < 2.0) {
                        document.getElementById("fingeringSelect0").options[16].disabled = true;
                        document.getElementById("fingeringSelect1").options[16].disabled = true;
                        document.getElementById("fingeringSelect2").options[16].disabled = true;

                        document.getElementById("fingeringSelect0").options[17].disabled = true;
                        document.getElementById("fingeringSelect1").options[17].disabled = true;
                        document.getElementById("fingeringSelect2").options[17].disabled = true;

                        document.getElementById("fingeringSelect0").options[18].disabled = true;
                        document.getElementById("fingeringSelect1").options[18].disabled = true;
                        document.getElementById("fingeringSelect2").options[18].disabled = true;
                    }




                } else if (data1 == MIDI_CC_111) {
                    document.getElementById("keySelect0").value = data2;
                } else if (data1 == MIDI_CC_112) {
                    document.getElementById("keySelect1").value = data2;
                } else if (data1 == MIDI_CC_113) {
                    document.getElementById("keySelect2").value = data2;
                } else if (data1 == MIDI_CC_116) {
                    lsb = data2;
                } else if (data1 == MIDI_CC_118) {
                    let sensorVal = parseInt((data2 << 7) | lsb); //receive pressure between 100 and 900
                    var x = (sensorVal - 100) * 24 / 900; //convert to inches of water.  105 is the approximate minimum sensor value.
                    p = Math.min(Math.max(p, 0), 24); //constrain
                    var p = x.toFixed(1); //round to 1 decimal
                    if (p < 0.2) {
                        p = 0
                    };

                    // Are we graphing pressure values?
                    if (gPressureGraphEnabled) {
                        capturePressure(p);
                    }
                    else {
                        document.getElementById("pressure").innerHTML = (p);
                        document.getElementById("pressure1").innerHTML = (p);
                        
                        if (document.getElementById("box8").style.display == "block") {
                            var barWidth = Math.min(1.0, p / 12) * 100; // this is only half the range
                            document.getElementById("exprInputPressureLevel").style.width = barWidth + "%";
                            let cents = calculateOutExprBendFromInput(sensorVal);
                            var centsBarWidth = cents/256.0;
                            let bendbar = document.getElementById("exprOutputBendLevel")
                            if (Math.abs(cents) < 0.5) {
                                // zero cents
                                bendbar.style.left = "49%";
                                bendbar.style.width = "2%";   
                            }
                            else if (cents > 0) {
                                bendbar.style.left = "50%";
                                bendbar.style.width = (centsBarWidth*100).toFixed(0) + "%";
                            } 
                            else {
                                bendbar.style.left = (50 + centsBarWidth*100).toFixed(0) + "%";
                                bendbar.style.width = (-centsBarWidth*100).toFixed(0) + "%";
                            }
                        }
                    }
                }

                for (var i = 0; i < numberOfGestures; i++) {
                    if (buttonRowWrite == i) {
                        if (data1 == MIDI_CC_106 && data2 <= MIDI_ACTION_MIDI_CHANNEL_END) {
                            document.getElementById("channel" + (buttonRowWrite)).value = data2;
                        }
                        if (data1 == MIDI_CC_107) {
                            document.getElementById("byte2_" + (buttonRowWrite)).value = data2;
                        }
                        if (data1 == MIDI_CC_108) {
                            document.getElementById("byte3_" + (buttonRowWrite)).value = data2;
                        }
                    }
                }




                if (data1 == MIDI_CC_106 && data2 > MIDI_ACTION_MIDI_CHANNEL_END) {

                    if (data2 >= MIDI_ENA_VIBRATO_HOLES_START && data2 <= MIDI_ENA_VIBRATO_HOLES_END) {
                        document.getElementById("vibratoCheckbox" + (data2 - MIDI_ENA_VIBRATO_HOLES_START)).checked = false;
                    }

                    if (data2 >= MIDI_DIS_VIBRATO_HOLES_START && data2 <= MIDI_DIS_VIBRATO_HOLES_END) {
                        document.getElementById("vibratoCheckbox" + (data2 - MIDI_DIS_VIBRATO_HOLES_START)).checked = true;
                    }

                    if (data2 == MIDI_STARTUP_CALIB) {
                        document.getElementById("calibrateradio0").checked = true;
                    }
                    if (data2 == MIDI_USE_LEARNED_CALIB) {
                        document.getElementById("calibrateradio1").checked = true;
                    }

                    if (data2 == MIDI_CC_106_VALUE_53) { //add an option for the uilleann regulators fingering pattern if a message is received indicating that it is supported.

                        for (i = 0; i < document.getElementById("fingeringSelect0").length; ++i) {
                            if (document.getElementById("fingeringSelect0").options[i].value == "9") {
                                var a = 1;
                            }
                        }

                        if (a != 1) {
                            var x = document.getElementById("fingeringSelect0");
                            var option = document.createElement("option");
                            option.text = "Uilleann regulators";
                            option.value = 9;
                            x.add(option);

                            var y = document.getElementById("fingeringSelect1");
                            var option = document.createElement("option");
                            option.text = "Uilleann regulators";
                            option.value = 9;
                            y.add(option);

                            var z = document.getElementById("fingeringSelect2");
                            var option = document.createElement("option");
                            option.text = "Uilleann regulators";
                            option.value = 9;
                            z.add(option);

                        }
                    }


                    if (data2 >= MIDI_WARBL2_SETTINGS_START && data2 <= MIDI_WARBL2_SETTINGS_END) {
                        WARBL2SettingsReceive = data2 - MIDI_WARBL2_SETTINGS_START;
                    }


                    if (data2 >= MIDI_BUTTON_ACTIONS_START && version > 3.9) {
						
						//console.log(buttonRowWrite); 

                        for (var j = 0; j < numberOfGestures; j++) { //update button configuration	
                            if (buttonRowWrite == j) {
                                for (var k = 0; k < 27; k++) {
                                    if (data2 == MIDI_BUTTON_ACTIONS_START + k) {
                                        document.getElementById("row" + (buttonRowWrite)).value = k;
                                    }
                                }
                            }
                        }
                    }

                } //end CC 106





                if (data1 == MIDI_CC_119) {
                    if (WARBL2SettingsReceive == 0) {
                        document.getElementById("WARBL2Radio" + data2).checked = true;
                    }
                    else if (WARBL2SettingsReceive == 1) {
                        document.getElementById("checkbox19").checked = data2;
                    }
                    else if (WARBL2SettingsReceive == 2) {
                        document.getElementById("poweroffSlider").value = data2;
                        document.getElementById("poweroffValue").innerHTML = data2;
                    }
                    else if (WARBL2SettingsReceive == 15) {
                        var x = (data2 + 50) / 100;
                        var p = x.toFixed(2); //round to 2 decimals
                        //p = Math.min(Math.max(p, 0), 24); //constrain
                        document.getElementById("voltage").innerHTML = p;
                    }
                    else if (WARBL2SettingsReceive == 16) {
                        if (data2 == 0) {
                            document.getElementById("charging").style.display = "none";
                        }
                        if (data2 == 1) {
                            document.getElementById("charging").style.display = "block";
                        }
                        if (data2 == 2) {
                            document.getElementById("chargingLabel").innerHTML = "Battery fault detected";
                            document.getElementById("chargingLabel").style.display = "block";
							setTimeout(turnOffFault, 20000);
                        }
                    }
                    else if (WARBL2SettingsReceive == 17) { //low byte of BLE connection interval
                        connIntvlLSB = data2;
                    }
                    else if (WARBL2SettingsReceive == 18) { //high byte of BLE connection interval
                        var x = parseInt((data2 << 7) | connIntvlLSB); 
                        x = x / 100; //convert back to a decimal
                        x = parseFloat(x.toFixed(4));
                        if (x == 0) {
                            document.getElementById("connLabel").innerHTML = "BLE not connected";
                            document.getElementById("connLabel").style.left = "95px";
                        }
                        else {
                            document.getElementById("connLabel").innerHTML = "BLE connected at " + x + " mS interval";
                            document.getElementById("connLabel").style.left = "55px";
                        }
                    }
                    else if (WARBL2SettingsReceive == 19) { //battery percentage	
                        var batt = data2;
                        document.getElementById("percentage").innerHTML = batt + "%";
                        if (batt > 87) {
                            document.getElementById("battery100").style.display = "block";
                            document.getElementById("battery75").style.display = "none";
                            document.getElementById("battery50").style.display = "none";
                            document.getElementById("battery25").style.display = "none";
                            document.getElementById("battery0").style.display = "none";
                        }
                        else if (batt > 63) {
                            document.getElementById("battery100").style.display = "none";
                            document.getElementById("battery75").style.display = "block";
                            document.getElementById("battery50").style.display = "none";
                            document.getElementById("battery25").style.display = "none";
                            document.getElementById("battery0").style.display = "none";
                        }
                        else if (batt > 37) {
                            document.getElementById("battery100").style.display = "none";
                            document.getElementById("battery75").style.display = "none";
                            document.getElementById("battery50").style.display = "block";
                            document.getElementById("battery25").style.display = "none";
                            document.getElementById("battery0").style.display = "none";
                        }
                        else if (batt > 13) {
                            document.getElementById("battery100").style.display = "none";
                            document.getElementById("battery75").style.display = "none";
                            document.getElementById("battery50").style.display = "none";
                            document.getElementById("battery25").style.display = "block";
                            document.getElementById("battery0").style.display = "none";
                        }
                        else {
                            document.getElementById("battery100").style.display = "none";
                            document.getElementById("battery75").style.display = "none";
                            document.getElementById("battery50").style.display = "none";
                            document.getElementById("battery25").style.display = "none";
                            document.getElementById("battery0").style.display = "block";
                        }


                    }





                }
            }
    }

updateCells(); //keep enabled/disabled cells updated.

}



function bit_test(num, bit) {
    return ((num >> bit) % 2 != 0)
}




function contract(gesture) {
	gesture.style.scale="1"
	gesture.style.color = "#c3c0c0";
}



function turnOffFault(){
	document.getElementById("chargingLabel").style.display = "none";
}




// MrMep: I don't think this applies to WARBL2
// I'm not modifying it
function sendCustomFingeringFill() {

    modalclose(22);

    var selection = document.getElementById("customFingeringFill").value;
    for (i = 1; i < 12; ++i) {

        document.getElementById("fingeringInput" + i).value = customFingeringFills[selection][i - 1];
        var x = document.getElementById("fingeringInput" + i).value;

        sendToWARBL(MIDI_CC_104, 86 + i);
        sendToWARBL(MIDI_CC_105, x);

    }
    document.getElementById("customFingeringFill").value = "12";

}






function sendWARBL2CustomFingeringFill() {
    modalclose(24);
    modal(25);
    var selection = document.getElementById("WARBL2customFingeringFill").value;
    selection = parseFloat(selection);

    var textArea = document.getElementById('WARBL2CustomTextArea');
    var lines = textArea.value.split('\n');    // lines is an array of strings

    // Loop through all lines
    for (var j = 0; j < lines.length; j++) {
        //console.log('Line ' + j + ' is ' + lines[j])

        //validate
        if (!(lines.length == 257 || lines.length == 256)) { //there may be a blank line at the end
            modalclose(25);
            alert("There must be 256 MIDI notes. Please try again.");
            document.getElementById('WARBL2customFingeringFill').value = '10';
            document.getElementById("WARBL2CustomTextArea").value = '';
            return;

        }
        if (lines[j] < 0 || lines[j] > 127 || isNaN(lines[j])) {
            modalclose(25);
            alert("Each MIDI note must be in the range of 0 to 127.");
            document.getElementById('WARBL2customFingeringFill').value = '10';
            document.getElementById("WARBL2CustomTextArea").value = '';
            return;
        }
    }
    //console.log(selection + 100);
    sendToWARBL(MIDI_CC_109, (selection + MIDI_CUSTOM_CHARTS_START));
    for (var k = 0; k < 256; k++) {
        sendToWARBL(MIDI_CC_105, lines[k]);
    }


    //modalclose(25);
}





// MrMep: I don't think this applies to WARBL2
// I'm not modifying it
function fingeringInput(input, selection) { 	//send the custom fingering input entry
    var x = document.getElementById("fingeringInput" + input).value;
    /*
    if (x < 0 || x > 127 || isNaN(x)) {
        alert("Value must be 0-127.");
        document.getElementById("fingeringInput" + input).value = null;
    }		
    else{
        */

    sendToWARBL(MIDI_CC_104, 86 + input);
    sendToWARBL(MIDI_CC_105, x);
    //}
}





function sendFingeringSelect(row, selection) {
    selection = parseFloat(selection);
    updateCells();
    updateCustom();
    blink(1);
    //default keys for original WARBL
	if (version < 4.0){
    var key;
    if (selection == 2) {
        key = 8;
    } //GHB
    else if (selection == 3) {
        key = 3;
    } //Northumbrian
    else if (selection == 5 || selection == 13) {
        key = 125;
    } //Gaita
    else if (selection == 6) {
        key = 122;
    } //NAF
    else if (selection == 8) {
        key = 125;
    } //Recorder
    else if (selection == 11) {
        key = 113;
    } //Xiao
    else if (selection == 12 || selection == 14 || selection == 15) {
        key = 123;
    } //Sax
    else if (selection == 17 || selection == 18) {
        key = 2;
    } //Sackpipa
    else if (selection == 20) {
        key = 123;
    } //Bombarde
    else if (selection == 22) {
        key = 5;
    } //Medieval bagpipes
    else {
        key = 0;
    } //default key of D for many patterns

	}
	else {key = 0;} // For WARBL2 always revert key (transpose) to 0 when a new fingerin chart is selected
	document.getElementById("keySelect" + row).value = key; //set key menu
    //send the fingering pattern
    sendToWARBL(MIDI_CC_102, MIDI_FINGERING_PATTERN_MODE_START + row);
    sendToWARBL(MIDI_CC_102, MIDI_FINGERING_PATTERN_START + selection);
    sendKey(row, key);

}





function defaultInstrument() { //tell WARBL to set the current instrument as the default

    defaultInstr = instrument;
    handleDefault();
    sendToWARBL(MIDI_CC_102, MIDI_DEFAULT_MODE_START);

}

function handleDefault() { //display correct default instrument and "set default" buttons	

    if (version > 1.6 || version == "Unknown") {

        if (defaultInstr != 0) {
            document.getElementById("instrument1Label").innerHTML = "Instrument 1:";
        }
        if (defaultInstr != 1) {
            document.getElementById("instrument2Label").innerHTML = "Instrument 2:";
        }
        if (defaultInstr != 2) {
            document.getElementById("instrument3Label").innerHTML = "Instrument 3:";
        }

        if (defaultInstr == 0) {
            document.getElementById("instrument1Label").innerHTML = "Instrument 1 (default):";
        }
        if (defaultInstr == 1) {
            document.getElementById("instrument2Label").innerHTML = "Instrument 2 (default):";
        }
        if (defaultInstr == 2) {
            document.getElementById("instrument3Label").innerHTML = "Instrument 3 (default):";
        }

        if (defaultInstr != instrument) {
            document.getElementById("defaultInstrument").style.visibility = "visible";

            if (instrument == 0) {
                document.getElementById("defaultInstrument").style.left = "130px";
            }
            if (instrument == 1) {
                document.getElementById("defaultInstrument").style.left = "430px";
            }
            if (instrument == 2) {
                document.getElementById("defaultInstrument").style.left = "730px";
            }
        } else {
            document.getElementById("defaultInstrument").style.visibility = "hidden";

        }
    }
}

function sendKey(row, selection) {
    selection = parseFloat(selection);
    row = parseFloat(row);
    updateCells();
    blink(1);
    sendToWARBL(MIDI_CC_111 + row, selection);
}

function sendFingeringRadio(tab) { //change instruments, showing the correct tab for each instrument.

    instrument = tab;
    updateCustom();
    advancedOkay(); //turn off the advanced tab
    pressureOkay();
    okayCCmap();
    okayIMUmap();
	okayPitchRegistermap();
    okayOverride();
    backIMU();
    backPressure();
    handleDefault(); //display correct default instrument and "set default" buttons	
    customFingeringOkay();
    if (tab == 0) {
        document.getElementById("instrument0").style.display = "block";
        document.getElementById("instrument1").style.display = "none";
        document.getElementById("instrument2").style.display = "none";

        document.getElementById("key0").style.display = "block";
        document.getElementById("key1").style.display = "none";
        document.getElementById("key2").style.display = "none";
        sendToWARBL(MIDI_CC_102, MIDI_CURRENT_MODE_START);
    } else if (tab == 1) {
        document.getElementById("instrument0").style.display = "none";
        document.getElementById("instrument1").style.display = "block";
        document.getElementById("instrument2").style.display = "none";

        document.getElementById("key0").style.display = "none";
        document.getElementById("key1").style.display = "block";
        document.getElementById("key2").style.display = "none";
        blink(2);
        sendToWARBL(MIDI_CC_102, MIDI_CURRENT_MODE_START +1);
    } else if (tab == 2) {
        document.getElementById("instrument0").style.display = "none";
        document.getElementById("instrument1").style.display = "none";
        document.getElementById("instrument2").style.display = "block";

        document.getElementById("key0").style.display = "none";
        document.getElementById("key1").style.display = "none";
        document.getElementById("key2").style.display = "block";
        blink(3);
        sendToWARBL(MIDI_CC_102, MIDI_CURRENT_MODE_START +2);
    }
    updateCells();
    updateCustom();
}


//move selection box for advanced overblowing settings based on whether bag or breath is selected
function updateSelected() {
    if (document.getElementById("bagBreath0").checked == true) {
        document.getElementById("selectedSettings").style.left = "125px";
        document.getElementById("selectedSettings").style.width = "162px";
    } else {
        document.getElementById("selectedSettings").style.left = "285px";
        document.getElementById("selectedSettings").style.width = "162px";
    }
}



function sendSenseDistance(selection) {
    blink(1);
    selection = parseFloat(selection);
    x = 0 - parseFloat(selection);
    sendToWARBL(MIDI_CC_103, x);
}


function sendDepth(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_117, selection);
}

function sendSlideLimit(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_SLIDE_LIMIT_MAX);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendShakeDepth(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_109, MIDI_Y_PITCHBEND_DEPTH);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendShakeVibModeOnCommand(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_109, MIDI_Y_PITCHBEND_MODE);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendAutoCenterYawInterval(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_109, MIDI_AUTOCENTER_YAW_INTERVAL);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendpitchRegisterNumber(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_109, MIDI_PITCH_REGISTER_NUMBER);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendExpressionDepth(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_DEPTH);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendExpressionCenterPressure(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_FIXED_CENTER_PRESSURE);
    sendToWARBL(MIDI_CC_105, selection);
    updateExpressionCenterPressureLabel();
}

function sendExpressionCurveLow(value) {
    blink(1);
    selection = parseFloat(value);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_CURVE_LOW);
    sendToWARBL(MIDI_CC_105, selection);
    updateExpressionCurveLabels();
}

function sendExpressionCurveHigh(value) {
    blink(1);
    selection = parseFloat(value);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_CURVE_HIGH);
    sendToWARBL(MIDI_CC_105, selection);
    updateExpressionCurveLabels();
}

function resetExpressionCurveLow() {
    var lowslider = document.getElementById('exprcurvelowslider');
    lowslider.value = 64;    
    sendExpressionCurveLow(64);
}

function resetExpressionCurveHigh() {
    var highslider = document.getElementById('exprcurvehighslider');
    highslider.value = 64;    
    sendExpressionCurveHigh(64);
}



function sendClampExpr(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_OUT_CLAMP);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendCurveRadio(selection) {
    blink(1);
    selection = parseFloat(selection);
    curve[mapSelection] = selection;
    if (mapSelection == 0) {
        sendToWARBL(MIDI_CC_104, MIDI_CURVE);
    }
    else {
        sendToWARBL(MIDI_CC_104, MIDI_VELOCITY_CURVE + mapSelection - 1);
    }
    sendToWARBL(MIDI_CC_105, selection);
}

function sendPressureChannel(selection) {
    blink(1);
    var x = parseFloat(selection);
    if (x < 1 || x > 16 || isNaN(x)) {
        alert("Value must be 1-16.");
        document.getElementById("pressureChannel").value = null;
    } else {
        sendToWARBL(MIDI_CC_104, MIDI_PRESSURE_CHANNEL);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendPressureCC(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_PRESSURE_CC);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendBendRange(selection) {
    blink(1);
    var x = parseFloat(selection);
    if (x < 1 || x > 96 || isNaN(x)) {
        alert("Value must be 1-96.");
        document.getElementById("midiBendRange").value = null;
    } else {
        sendToWARBL(MIDI_CC_104, MIDI_BEND_RANGE);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendNoteChannel(selection) {
    blink(1);
    var x = parseFloat(selection);
    if (x < 1 || x > 16 || isNaN(x)) {
        alert("Value must be 1-16.");
        document.getElementById("noteChannel").value = null;
    } else {
        sendToWARBL(MIDI_CC_104, MIDI_MIDI_CHANNEL);
        sendToWARBL(MIDI_CC_105, x);
    }
}


//pressure input slider
slider.noUiSlider.on('change', function (values) {
    blink(1);
    if (mapSelection == 0) {
        inputSliderMin[0] = parseInt(values[0]);
        inputSliderMax[0] = parseInt(values[1]);
        sendToWARBL(MIDI_CC_104, MIDI_INPUT_PRESSURE_MIN);
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_INPUT_PRESSURE_MAX);
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));
    }
    else {
        inputSliderMin[mapSelection] = parseInt(values[0]);
        inputSliderMax[mapSelection] = parseInt(values[1]);
        sendToWARBL(MIDI_CC_104, MIDI_ED_VARS2_START + ((mapSelection - 1) * 4));
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_ED_VARS2_START + 1 + ((mapSelection - 1) * 4));
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));
    }
});

//pressure output slider
slider2.noUiSlider.on('change', function (values) {
    blink(1);
    if (mapSelection == 0) {
        outputSliderMin[0] = parseInt(values[0]);
        outputSliderMax[0] = parseInt(values[1]);
        sendToWARBL(MIDI_CC_104, MIDI_OUTPUT_PRESSURE_MIN);
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_OUTPUT_PRESSURE_MAX);
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));
    }
    else {
        outputSliderMin[mapSelection] = parseInt(values[0]);
        outputSliderMax[mapSelection] = parseInt(values[1]);
        sendToWARBL(MIDI_CC_104, MIDI_ED_VARS2_START +2 + ((mapSelection - 1) * 4));
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_ED_VARS2_START +3 + ((mapSelection - 1) * 4));
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));
    }
});

//expression override slider
exprlowslider.noUiSlider.on('change', function (values) {
    blink(1);
    
    if (version < 4.3) {
        sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MIN);
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MAX);
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));    
    }
    else {
        sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MIN);
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
        sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MIN_HIGH);
        sendToWARBL(MIDI_CC_105, parseInt(values[1]));
        console.log("lowchange");
        // if the max is > exprhighslider.min, set the other to match
        //console.log("exprlowslider values " + values[0] + " " + values[1]);
        if ( parseInt(values[1]) > parseInt(exprhighslider.noUiSlider.get()[0])) {
            console.log("  exprhigh values " + exprhighslider.noUiSlider.get()[0] + " " + exprhighslider.noUiSlider.get()[1]);
            exprhighslider.noUiSlider.set([parseInt(values[1]), null]);
            sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MAX_LOW);
            sendToWARBL(MIDI_CC_105, parseInt(values[1]));
        }
    }
});

exprhighslider.noUiSlider.on('change', function (values) {
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MAX_LOW);
    sendToWARBL(MIDI_CC_105, parseInt(values[0]));
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MAX);
    sendToWARBL(MIDI_CC_105, parseInt(values[1]));

    console.log("highchange");
    
    // if the min is < slider3.max, set the other to match
    if (parseInt(values[0]) < parseInt(exprlowslider.noUiSlider.get()[1])) {
        exprlowslider.noUiSlider.set([null, parseInt(values[0])]);
        console.log("  exprlow values " + exprlowslider.noUiSlider.get()[0] + " " + exprlowslider.noUiSlider.get()[1]);
        
        sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_MIN_HIGH);
        sendToWARBL(MIDI_CC_105, parseInt(values[0]));
    }
});

exprlowbendslider.noUiSlider.on('change', function (values) {
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_OUT_LOW_CENTS);
    sendToWARBL(MIDI_CC_105, parseInt(values[0]));
});

exprhighbendslider.noUiSlider.on('change', function (values) {
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_EXPRESSION_OUT_HIGH_CENTS);
    sendToWARBL(MIDI_CC_105, parseInt(values[0]));
});


slider5.noUiSlider.on('change', function (values, handle) {

    if (center[IMUmapSelection] && IMUmapSelection != 2) { //mirror IMU input range if the "center" switch is selected. Using the "change" event instead of "slide" means that the second one won't be updated until the first one is released, but otherwise it seems sluggish.
        var min = parseFloat(values[handle]).toFixed(0);
        if (handle) {
            if (min > 0) {
                slider5.noUiSlider.set([-min, null]);
            }
            else {
                slider5.noUiSlider.set([0, 0]);
            }
        } else {
            if (min < 0) {
                slider5.noUiSlider.set([null, -min]);
            }
            else {
                slider5.noUiSlider.set([0, 0]);
            }
        }
    }
    blink(1);

    var handles = slider5.noUiSlider.get();

    inputSliderMin[IMUmapSelection + 3] = parseInt(handles[0]);
    inputSliderMax[IMUmapSelection + 3] = parseInt(handles[1]);
	
	//console.log(inputSliderMin[IMUmapSelection + 3]); 

    sendToWARBL(MIDI_CC_109, ((IMUmapSelection) * 4 + 1));
    if (IMUmapSelection == 3) {
        sendToWARBL(MIDI_CC_105, (parseInt(handles[0]) + 90) / 5); //convert to 1-36 steps to send
    }
    else {
        sendToWARBL(MIDI_CC_105, (parseInt(handles[0]) + 90) / 5);
    }
    sendToWARBL(MIDI_CC_109, ((IMUmapSelection) * 4) + 2);

    if (IMUmapSelection == 3) {
        sendToWARBL(MIDI_CC_105, (parseInt(handles[1]) + 90) / 5);
    }
    else {
        sendToWARBL(MIDI_CC_105, (parseInt(handles[1]) + 90) / 5);
    }

});



slider4.noUiSlider.on('change', function (values, handle) {

    outputSliderMin[IMUmapSelection + 3] = parseInt(values[0]);
    outputSliderMax[IMUmapSelection + 3] = parseInt(values[1]);

    sendToWARBL(MIDI_CC_109, ((IMUmapSelection) * 4 + 3));
    sendToWARBL(MIDI_CC_105, parseInt(values[0]));
    sendToWARBL(MIDI_CC_109, ((IMUmapSelection) * 4 + 4));
    sendToWARBL(MIDI_CC_105, parseInt(values[1]));


});



slider.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('slider-value-min'),
        marginMax = document.getElementById('slider-value-max');

    if (handle) {
        var min = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMax.innerHTML = min;
    } else {
        var max = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMin.innerHTML = max;
    }

});

slider2.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('slider2-value-min'),
        marginMax = document.getElementById('slider2-value-max');
    if (handle) {
        marginMax.innerHTML = parseInt(values[handle]);
    } else {
        marginMin.innerHTML = parseInt(values[handle]);
    }
});

exprlowslider.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('exprlowslider-value-min'),
        marginMax = document.getElementById('exprlowslider-value-max');

    if (handle) {
        var min = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMax.innerHTML = min;
    } else {
        var max = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMin.innerHTML = max;
    }

});

exprhighslider.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('exprhighslider-value-min'),
        marginMax = document.getElementById('exprhighslider-value-max');

    if (handle) {
        var min = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMax.innerHTML = min;
    } else {
        var max = parseFloat(values[handle] * 0.24).toFixed(1);
        marginMin.innerHTML = max;
    }

});

exprlowbendslider.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('exprlowbendslider-value');

    var min = parseFloat(2 * (values[0] - 64)).toFixed(0);
    marginMin.innerHTML = min;
    
});

exprhighbendslider.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('exprhighbendslider-value');

    var min = parseFloat(2 * (values[0] - 64)).toFixed(0);
    marginMin.innerHTML = min;
});

slider4.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('slider4-value-min'),
        marginMax = document.getElementById('slider4-value-max');
    if (handle) {
        marginMax.innerHTML = parseInt(values[handle]);
    } else {
        marginMin.innerHTML = parseInt(values[handle]);
    }

    var handles4 = slider4.noUiSlider.get();
    elements = document.getElementById("slider4").getElementsByClassName("noUi-connect");

    if (parseInt(handles4[0]) > parseInt(handles4[1])) {
        for (var i = 0; i < elements.length; i++) {
            elements[i].style.backgroundColor = "#f7c839";
        }
    }

    else {
        for (var i = 0; i < elements.length; i++) {
            elements[i].style.backgroundColor = "#262626";
        }
    }


});


slider5.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('slider5-value-min'),
        marginMax = document.getElementById('slider5-value-max');
    if (IMUmapSelection == 3) {
        if (handle) {
            var min = parseFloat(values[handle] * 2).toFixed(0);
            marginMax.innerHTML = min;
        } else {
            var max = parseFloat(values[handle] * 2).toFixed(0);
            marginMin.innerHTML = max;
        }
    }
    else {
        if (handle) {
            marginMax.innerHTML = parseInt(values[handle]);
        } else {
            marginMin.innerHTML = parseInt(values[handle]);
        }

    }
});

slider6.noUiSlider.on('update', function (values, handle) {
    var marginMin = document.getElementById('slider6-value-min'),
        marginMax = document.getElementById('slider6-value-max');
        if (handle) {
            marginMax.innerHTML = parseInt(values[handle]);
        } else {
            marginMin.innerHTML = parseInt(values[handle]);
    }
});


slider6.noUiSlider.on('change', function (values, handle) {
    blink(1);
	 var handles = slider6.noUiSlider.get();
    inputSliderMin[IMUmapSelection + 3] = parseInt(handles[0]);
    inputSliderMax[IMUmapSelection + 3] = parseInt(handles[1]);
    sendToWARBL(MIDI_CC_109, MIDI_PITCH_REGISTER_INPUT_MIN);
    sendToWARBL(MIDI_CC_105, (parseInt(handles[0]) + 90) / 5); //convert to 1-36 steps to send
    sendToWARBL(MIDI_CC_109, MIDI_PITCH_REGISTER_INPUT_MAX);
    sendToWARBL(MIDI_CC_105, (parseInt(handles[1]) + 90) / 5); //convert to 1-36 steps to send
});




function sendRoll(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_SEND_ROLL);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendPitch(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_SEND_PITCH);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendYaw(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_SEND_YAW);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendShake(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_Y_SHAKE_PITCHBEND);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendAutoCenterYaw(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_AUTOCENTER_YAW);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendPitchRegister(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_109, MIDI_PITCH_REGISTER);
    sendToWARBL(MIDI_CC_105, selection);
}


function sendDronesOnCommand(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_DRONES_ON_COMMAND);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendDronesOnChannel(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 16 || isNaN(x)) {
        alert("Value must be 1-16.");
        document.getElementById("dronesOnChannel").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_ON_CHANNEL);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesOnByte2(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 127 || isNaN(x)) {
        alert("Value must be 0 - 127.");
        document.getElementById("dronesOnByte2").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_ON_BYTE2);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesOnByte3(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 127 || isNaN(x)) {
        alert("Value must be 0 - 127.");
        document.getElementById("dronesOnByte3").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_ON_BYTE3);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesOffCommand(selection) {
    blink(1);
    var y = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_DRONES_OFF_COMMAND);
    sendToWARBL(MIDI_CC_105, y);
}

function sendDronesOffChannel(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 16 || isNaN(x)) {
        alert("Value must be 1-16.");
        document.getElementById("dronesOffChannel").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_OFF_CHANNEL);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesOffByte2(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 127 || isNaN(x)) {
        alert("Value must be 0 - 127.");
        document.getElementById("dronesOffByte2").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_OFF_BYTE2);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesOffByte3(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 127 || isNaN(x)) {
        alert("Value must be 0 - 127.");
        document.getElementById("dronesOffByte3").value = null;
    } else {
        blink(1);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_OFF_BYTE3);
        sendToWARBL(MIDI_CC_105, x);
    }
}

function sendDronesRadio(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_104, MIDI_DRONES_CONTROL_MODE);
    sendToWARBL(MIDI_CC_105, selection);
}

function learnDrones() {
    blink(1);
    document.getElementById("dronesPressureInput").style.backgroundColor = "#32CD32";
    setTimeout(blinkDrones, 500);
    sendToWARBL(MIDI_CC_106, MIDI_LEARN_DRONES_PRESSURE);

}

function blinkDrones() {
    document.getElementById("dronesPressureInput").style.backgroundColor = "#FFF";
}


function blinkOctave() {
    document.getElementById("octavePressureInput").style.backgroundColor = "#FFF";
}

function sendDronesPressure(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 24 || isNaN(x)) {
        alert("Value must be 0.0 - 24.0.");
        document.getElementById("dronesPressureInput").value = null;
    } else {
        x = parseInt(x * 900 / 24 + 100);
        blink(1);
        document.getElementById("dronesPressureInput").style.backgroundColor = "#32CD32";
        setTimeout(blinkDrones, 500);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_PRESSURE_LOW_BYTE);
        sendToWARBL(MIDI_CC_105, x & 0x7F);
        sendToWARBL(MIDI_CC_104, MIDI_DRONES_PRESSURE_HIGH_BYTE);
        sendToWARBL(MIDI_CC_105, x >> 7);
    }
}

function sendOctavePressure(selection) {
    var x = parseFloat(selection);
    if (x < 0 || x > 24 || isNaN(x)) {
        alert("Value must be 0.0 - 24.0.");
        document.getElementById("octavePressureInput").value = null;
    } else {
        x = parseInt(x * 900 / 24 + 100);
        blink(1);
        document.getElementById("octavePressureInput").style.backgroundColor = "#32CD32";
        setTimeout(blinkOctave, 500);
        sendToWARBL(MIDI_CC_104, MIDI_LEARNED_PRESS_LSB);
        sendToWARBL(MIDI_CC_105, x & 0x7F);
        sendToWARBL(MIDI_CC_104, MIDI_LEARNED_PRESS_MSB);
        sendToWARBL(MIDI_CC_105, x >> 7);
    }

}

function learn() {
    blink(1);
    document.getElementById("octavePressureInput").style.backgroundColor = "#32CD32";
    setTimeout(blinkOctave, 500);
    sendToWARBL(MIDI_CC_106, MIDI_LEARN_INITIAL_NOTE_PRESS);
}

function sendCalibrateRadio(selection) {
    selection = parseFloat(selection);
    blink(1);
    sendToWARBL(MIDI_CC_106, MIDI_STARTUP_CALIB + selection);
}

function sendPitchbendRadio(selection) {
    updateCustom();
    updateCustom();
    selection = parseFloat(selection);
    if (selection > 0) {
        blink(selection + 1);
    }
    sendToWARBL(MIDI_CC_102, MIDI_PB_MODE_START + selection);
}

function sendVibratoHoles(holeNumber, selection) {
    selection = +selection; //convert true/false to 1/0
    sendToWARBL(MIDI_CC_106, (MIDI_DIS_VIBRATO_HOLES_START - (selection * 10)) + holeNumber);
}

function updateCustom() { //keep correct settings enabled/disabled with respect to the custom vibrato switch and send pressure as CC switches.


    var a = document.getElementById("fingeringSelect" + instrument).value;

    if (document.getElementById("pitchbendradio2").checked == false && (((a == 0 || a == 1 || a == 4 || a == 10) && ((document.getElementById("pitchbendradio1").checked == true && version < 1.6) || (version == "Unknown" || version > 1.5))) || (version == "Unknown" || version > 1.5) && (a == 3 || a == 2))) {
        document.getElementById("checkbox5").disabled = false;
        document.getElementById("switch5").style.cursor = "pointer";
    } else {
        //document.getElementById("checkbox5").checked = false;
        document.getElementById("checkbox5").disabled = true;
        document.getElementById("switch5").style.cursor = "default";
        //sendToWARBL(MIDI_CC_104,44); //if custom is disabled, tell WARBL to turn it off
        //sendToWARBL(MIDI_CC_105, 0);
        //blink(1);
    }
    if (document.getElementById("checkbox5").checked == true && document.getElementById("checkbox5").disabled == false) {
        for (var i = 0; i < 9; i++) {
            document.getElementById("vibratoCheckbox" + i).disabled = true;
            document.getElementById("vibratoCheckbox" + i).style.cursor = "default";
        }
    } else {
        for (var i = 0; i < 9; i++) {
            document.getElementById("vibratoCheckbox" + i).disabled = false;
            document.getElementById("vibratoCheckbox" + i).style.cursor = "pointer";
        }
    }

    if (document.getElementById("checkbox18").checked == true) {
        document.getElementById("fingeringInput11").disabled = true;
        document.getElementById("fingeringInput11").style.cursor = "default";

    } else {

        document.getElementById("fingeringInput11").disabled = false;
        document.getElementById("fingeringInput11").style.cursor = "pointer";

    }
    
    if (version >= 4.3) {
        // new pitch expression uses smaller range
        exprlowslider.noUiSlider.updateOptions({
            range: {
                'min': 0,
                'max': 50
            }, start: [0, 50]
        });
    } else {
        exprlowslider.noUiSlider.updateOptions({
            range: {
                'min': 0,
                'max': 100
            }, start: [0, 100]
        });        
    }
}

function sendBreathmodeRadio(selection) {
    selection = parseFloat(selection);
    if (selection > 0) {
        blink(selection + 1);
    }
    if (document.getElementById("sensorradio1").checked == true) { //update override
        document.getElementById("checkbox16").disabled = true;
        document.getElementById("overrideExprCheck").disabled = true;
    }
    else {
        document.getElementById("checkbox16").disabled = false;
        document.getElementById("overrideExprCheck").disabled = false;
    }
    sendToWARBL(MIDI_CC_102, MIDI_BREATH_MODE_START + selection);

    updateExpressionSliderEnableState();

}

function advanced() {
    document.getElementById("box2").style.display = "block";
    document.getElementById("box1").style.display = "none";
    updateSelected();
}

function advancedOkay() {
    document.getElementById("box2").style.display = "none";
    document.getElementById("box1").style.display = "block";
}

function advancedPB() {
    if (version > 1.8 || version == "Unknown") {
        document.getElementById("box4").style.display = "block";
        document.getElementById("box5").style.display = "none";
    }
}

function mapPressure() {
    document.getElementById("expressionPressureBox").style.display = "block";
    document.getElementById("box9").style.display = "none";
}


function mapIMU() {
    document.getElementById("box10").style.display = "block";
    document.getElementById("box9").style.display = "none";
}


function backPressure() {
    if (version > 3.9 || version == "Unknown") {
        document.getElementById("expressionPressureBox").style.display = "none";
        document.getElementById("box9").style.display = "block";
    }
}


function backIMU() {

    document.getElementById("box10").style.display = "none";
    if (version > 3.9 || version == "Unknown") {
        document.getElementById("box9").style.display = "block";
    }
}


function overRideExpression() {
    if (version > 1.8 || version == "Unknown") {
        if (document.getElementById("sensorradio1").checked == true) {
            document.getElementById("checkbox16").disabled = true;
            document.getElementById("overrideExprCheck").disabled = true;
        }
        else {
            document.getElementById("checkbox16").disabled = false;
            document.getElementById("overrideExprCheck").disabled = false;
        }
        
        
        var dispval = version < 4.3 ? "none" : "block";
        // hide new advanced pressure overridedispval
        document.getElementById("exprhighslider").style.display = dispval;
        document.getElementById("exprhighslider-value-min").style.display = dispval;                        
        document.getElementById("exprhighslider-value-max").style.display = dispval;                        
        document.getElementById("exprlowbendslider").style.display = dispval;
        document.getElementById("exprlowbendslider-value").style.display = dispval;                        
        document.getElementById("exprhighbendslider").style.display = dispval;
        document.getElementById("exprhighbendslider-value").style.display = dispval;                        
        document.getElementById("clampExprMaxLabel").style.display = dispval;
        document.getElementById("clampExprSwitch").style.display = dispval;
        document.getElementById("resetExpressionButton").style.display = dispval;
        document.getElementById("overrideRangeLabel").style.display = dispval;
        document.getElementById("overrideCentsLabel").style.display = dispval;
        document.getElementById("lowrangelabel").style.display = dispval;
        document.getElementById("lowrangelabel2").style.display = dispval;
        document.getElementById("highrangelabel").style.display = dispval;
        document.getElementById("highrangelabel2").style.display = dispval;
        document.getElementById("exprInputPressureLevelContainer").style.display = dispval;
        document.getElementById("exprcurvelowlabel").style.display = dispval;
        document.getElementById("exprcurvelowslider").style.display = dispval;
        document.getElementById("exprcurvelowslider-value").style.display = dispval;                        
        document.getElementById("exprcurvehighlabel").style.display = dispval;
        document.getElementById("exprcurvehighslider").style.display = dispval;
        document.getElementById("exprcurvehighslider-value").style.display = dispval;                        
        
        
        document.getElementById("box8").style.display = "block";
        document.getElementById("expressionPressureBox").style.display = "none";
    }
}

function okayOverride() {
    document.getElementById("box8").style.display = "none";
    document.getElementById("expressionPressureBox").style.display = "block";
}


function advancedOkayPB() {
    document.getElementById("box4").style.display = "none";
    document.getElementById("box5").style.display = "block";
}

function configureCustomFingering() {

    if (version < 3.9) {  //original WARBL custom fingering
        document.getElementById("topControls").style.display = "none";
        document.getElementById("customControls").style.display = "block";
        document.getElementById("box1").style.top = "740px";
        document.getElementById("box2").style.top = "740px";
        document.getElementById("box4").style.top = "740px";
        document.getElementById("box5").style.top = "740px";
        document.getElementById("pressuregraph").style.top = "740px";
        document.getElementById("box3").style.top = "1200px";
        document.getElementById("expressionPressureBox").style.top = "1200px";
        document.getElementById("box9").style.top = "1200px";
        document.getElementById("box10").style.top = "1200px";
        document.getElementById("box7").style.top = "1200px";
        document.getElementById("box8").style.top = "1200px";
        document.getElementById("buttonBox").style.top = "1690px";
        document.getElementById("topcontrolbox").style.height = "2155px";
    }


    else { //WARBL2 custom fingering
        document.getElementById("topControls").style.display = "none";
        document.getElementById('WARBL2customFingeringFill').value = '10';
        document.getElementById("WARBL2CustomTextArea").value = '';
        document.getElementById("WARBL2customControls").style.display = "block";
    }
}

function customFingeringOkay() {
    document.getElementById("customControls").style.display = "none";
    document.getElementById("WARBL2customControls").style.display = "none";
    document.getElementById("topControls").style.display = "block";
    document.getElementById("box1").style.top = "440px";
    document.getElementById("box2").style.top = "440px";
    document.getElementById("box4").style.top = "440px";
    document.getElementById("box5").style.top = "440px";
    document.getElementById("pressuregraph").style.top = "440px";
    document.getElementById("box3").style.top = "900px";
    document.getElementById("expressionPressureBox").style.top = "900px";
    document.getElementById("box9").style.top = "900px";
    document.getElementById("box10").style.top = "900px";
    document.getElementById("box7").style.top = "900px";
    document.getElementById("box8").style.top = "900px";
    document.getElementById("buttonBox").style.top = "1390px";
    document.getElementById("topcontrolbox").style.height = "1855px";
    document.getElementById("customFingeringFill").value = "12";
}


function mapCC() {
    mapSelection = 0;
	document.getElementById('receivedpressureCC').innerHTML = null;
    document.getElementById("CCpressurelevel").style.width = 0 + "%";
    slider.noUiSlider.set([inputSliderMin[0], inputSliderMax[0]]);
    slider2.noUiSlider.set([outputSliderMin[0], outputSliderMax[0]]);
    document.getElementById("pressureChannel").style.visibility = "visible";
    document.getElementById("pressureCC").style.visibility = "visible";
    document.getElementById("expressionChannel").style.visibility = "visible";
    document.getElementById("highByte").style.visibility = "visible";
    document.getElementById("box7").style.display = "block";
    document.getElementById("expressionPressureBox").style.display = "none";
    if (curve[0] < 3) {
        document.getElementById("curveRadio" + curve[0]).checked = true;
    }
    document.getElementById("pressureMappingHeader").innerHTML = "CC Mapping";
}

function mapVelocity() {
    mapSelection = 1;
	document.getElementById('receivedpressureCC').innerHTML = null;
    document.getElementById("CCpressurelevel").style.width = 0 + "%";
    slider.noUiSlider.set([inputSliderMin[1], inputSliderMax[1]]);
    slider2.noUiSlider.set([outputSliderMin[1], outputSliderMax[1]]);
    if (curve[1] < 3) {
        document.getElementById("curveRadio" + curve[1]).checked = true;
    }
    document.getElementById("box7").style.display = "block";
    document.getElementById("expressionPressureBox").style.display = "none";
    document.getElementById("pressureMappingHeader").innerHTML = "Velocity Mapping";
    //console.log(mapSelection);
}

function mapAftertouch() {
    mapSelection = 2;
	document.getElementById('receivedpressureCC').innerHTML = null;
    document.getElementById("CCpressurelevel").style.width = 0 + "%";
    slider.noUiSlider.set([inputSliderMin[2], inputSliderMax[2]]);
    slider2.noUiSlider.set([outputSliderMin[2], outputSliderMax[2]]);
    if (curve[2] < 3) {
        document.getElementById("curveRadio" + curve[2]).checked = true;
    }
    document.getElementById("box7").style.display = "block";
    document.getElementById("expressionPressureBox").style.display = "none";
    document.getElementById("pressureMappingHeader").innerHTML = "Channel Pressure Mapping";
}

function mapPoly() {
    mapSelection = 3;
	document.getElementById('receivedpressureCC').innerHTML = null;
    document.getElementById("CCpressurelevel").style.width = 0 + "%";
    slider.noUiSlider.set([inputSliderMin[3], inputSliderMax[3]]);
    slider2.noUiSlider.set([outputSliderMin[3], outputSliderMax[3]]);
    if (curve[3] < 3) {
        document.getElementById("curveRadio" + curve[3]).checked = true;
    }
    document.getElementById("box7").style.display = "block";
    document.getElementById("expressionPressureBox").style.display = "none";
    document.getElementById("pressureMappingHeader").innerHTML = "Key Pressure Mapping";
}


function okayCCmap() {
    mapSelection = 4;
    document.getElementById("box7").style.display = "none";
    document.getElementById("expressionPressureBox").style.display = "block";
    document.getElementById("pressureChannel").style.visibility = "hidden";
    document.getElementById("pressureCC").style.visibility = "hidden";
    document.getElementById("expressionChannel").style.visibility = "hidden";
    document.getElementById("highByte").style.visibility = "hidden";
}




function mapRoll() {
    IMUmapSelection = 1;
    document.getElementById('receivedCC').innerHTML = null;
    document.getElementById("CClevel").style.width = 0 + "%";
    document.getElementById("checkbox23").checked = center[1];
    document.getElementById("IMUChannelInput").value = IMUchannel[0];
    document.getElementById("IMUCC").value = IMUnumber[0];
    slider5.noUiSlider.set([inputSliderMin[4], inputSliderMax[4]]);
    slider4.noUiSlider.set([outputSliderMin[4], outputSliderMax[4]]);
    document.getElementById("box10").style.display = "none";
    document.getElementById("box11").style.display = "block";
    document.getElementById("IMUMappingHeader").innerHTML = "Roll Mapping";
    document.getElementById("centerLabel").style.display = "block";
    document.getElementById("switch23").style.display = "block";
}

function mapPitch() {
    IMUmapSelection = 2;
    document.getElementById('receivedCC').innerHTML = null;
    document.getElementById("CClevel").style.width = 0 + "%";
    document.getElementById("IMUChannelInput").value = IMUchannel[1];
    document.getElementById("IMUCC").value = IMUnumber[1];
    slider5.noUiSlider.set([inputSliderMin[5], inputSliderMax[5]]);
    slider4.noUiSlider.set([outputSliderMin[5], outputSliderMax[5]]);
    document.getElementById("box10").style.display = "none";
    document.getElementById("box11").style.display = "block";
    document.getElementById("IMUMappingHeader").innerHTML = "Elevation Mapping";
    document.getElementById("centerLabel").style.display = "none";
    document.getElementById("switch23").style.display = "none";
}

function mapYaw() {
    IMUmapSelection = 3;

    document.getElementById("YawCenterControls").style.display = "block";
    document.getElementById("IMUMappingControls").style.top = "-190px";
    document.getElementById('receivedCC').innerHTML = null;
    document.getElementById("CClevel").style.width = 0 + "%";
    document.getElementById("IMUChannelInput").value = IMUchannel[2];
    document.getElementById("IMUCC").value = IMUnumber[2];
    document.getElementById("checkbox23").checked = center[3];
    slider5.noUiSlider.set([inputSliderMin[6], inputSliderMax[6]]);
    slider4.noUiSlider.set([outputSliderMin[6], outputSliderMax[6]]);
    document.getElementById("box10").style.display = "none";
    document.getElementById("box11").style.display = "block";
    document.getElementById("IMUMappingHeader").innerHTML = "Yaw Mapping";
    document.getElementById("centerLabel").style.display = "block";
    document.getElementById("switch23").style.display = "block";
}


function mapPitchRegister() {
    document.getElementById('receivedCC').innerHTML = null;
    document.getElementById("CClevel").style.width = 0 + "%";
    document.getElementById("IMUChannelInput").value = IMUchannel[1];
    document.getElementById("IMUCC").value = IMUnumber[1];
    slider5.noUiSlider.set([inputSliderMin[5], inputSliderMax[5]]);
    slider4.noUiSlider.set([outputSliderMin[5], outputSliderMax[5]]);
    document.getElementById("box10").style.display = "none";
    document.getElementById("box12").style.display = "block";
    document.getElementById("IMUMappingHeader").innerHTML = "Pitch Mapping";
    document.getElementById("centerLabel").style.display = "none";
    document.getElementById("switch23").style.display = "none";
}


function okayIMUmap() {
    document.getElementById("YawCenterControls").style.display = "none";
    document.getElementById("IMUMappingControls").style.top = "-170px";
    document.getElementById("box11").style.display = "none";
    document.getElementById("box10").style.display = "block";
}


function okayPitchRegistermap() {
    document.getElementById("box12").style.display = "none";
    document.getElementById("box10").style.display = "block";
}


function sendCenter(selection) {
    selection = +selection; //convert true/false to 1/0
    center[IMUmapSelection] = selection;
    blink(1);
    if (IMUmapSelection == 1) {
        sendToWARBL(MIDI_CC_109, MIDI_SEND_CENTER_ROLL);
    }
    else {
        sendToWARBL(MIDI_CC_109, MIDI_SEND_CENTER_YAW);
    }
    sendToWARBL(MIDI_CC_105, selection);
}

function sendIMUChannel(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    IMUchannel[IMUmapSelection - 1] = selection;
    sendToWARBL(MIDI_CC_109, IMUmapSelection - 1 + MIDI_IMU_CHANNEL_START);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendIMUCC(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    IMUnumber[IMUmapSelection - 1] = selection;
    sendToWARBL(MIDI_CC_109, IMUmapSelection -1 + MIDI_IMU_CC_START);
    sendToWARBL(MIDI_CC_105, selection);
}


function pressureGraph() {


    // Show the pressure graph
    document.getElementById("pressuregraph").style.display = "block";
    document.getElementById("box1").style.display = "none";

    // Start graphing the incoming pressure
    startPressureGraph();
}

function pressureOkay() {

    // Stop graphing the incoming pressure
    stopPressureGraph();

    // Hide the pressure
    document.getElementById("pressuregraph").style.display = "none";
    document.getElementById("box1").style.display = "block";
}

function advancedDefaults() {
    if (version > 2.0 || version == "Unknown") {
        document.getElementById("jumpFactor7").value = "3";
        document.getElementById("jumpFactor8").value = "7";
        document.getElementById("jumpFactor9").value = "20";
        document.getElementById("jumpFactor11").value = "12";
        document.getElementById("jumpFactor12").value = "50";
        for (var i = 7; i < 10; i++) {
            var k = document.getElementById("jumpFactor" + (i));
            k.dispatchEvent(new Event('input'));
            var j = document.getElementById("jumpFactor" + (i)).value;
            sendJumpFactor(i, j);
        }
        for (var i = 11; i < 13; i++) {
            var k = document.getElementById("jumpFactor" + (i));
            k.dispatchEvent(new Event('input'));
            var j = document.getElementById("jumpFactor" + (i)).value;
            sendJumpFactor(i, j);
        }
    }

    else {
        document.getElementById("jumpFactor1").value = "25";
        document.getElementById("jumpFactor2").value = "15";
        document.getElementById("jumpFactor3").value = "15";
        document.getElementById("jumpFactor4").value = "15";
        document.getElementById("jumpFactor5").value = "30";
        document.getElementById("jumpFactor6").value = "60";
        document.getElementById("jumpFactor7").value = "3";
        document.getElementById("jumpFactor8").value = "7";
        document.getElementById("jumpFactor9").value = "100";
        document.getElementById("jumpFactor10").value = "7";
        document.getElementById("jumpFactor11").value = "9";
        document.getElementById("jumpFactor12").value = "9";

        for (var i = 1; i < 13; i++) {
            var k = document.getElementById("jumpFactor" + (i));
            k.dispatchEvent(new Event('input'));
            var j = document.getElementById("jumpFactor" + (i)).value;
            sendJumpFactor(i, j);
        }
    }
}

function advancedBagDefaults() {


    if (version > 2.0 || version == "Unknown") {
        document.getElementById("jumpFactor1").value = "50";
        document.getElementById("jumpFactor2").value = "20";
        document.getElementById("jumpFactor3").value = "20";
        document.getElementById("jumpFactor4").value = "50";
        document.getElementById("jumpFactor6").value = "75";

        for (var i = 1; i < 7; i++) {
            var k = document.getElementById("jumpFactor" + (i));
            k.dispatchEvent(new Event('input'));
            var j = document.getElementById("jumpFactor" + (i)).value;
            sendJumpFactor(i, j);
        }
    }


    else {
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


        for (var i = 1; i < 13; i++) {
            var k = document.getElementById("jumpFactor" + (i));
            k.dispatchEvent(new Event('input'));
            var j = document.getElementById("jumpFactor" + (i)).value;
            sendJumpFactor(i, j);

        }
    }
}


function sendJumpFactor(factor, selection) {
    selection = parseFloat(selection);
    blink(1);
    sendToWARBL(MIDI_CC_104, factor);
    sendToWARBL(MIDI_CC_105, selection);
}


function sendRow(rowNum) {
    blink(1);
    updateCells();
    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + rowNum);
    var y = (100) + parseFloat(document.getElementById("row" + rowNum).value);
    if (version < 4.0) {
        sendToWARBL(MIDI_CC_102, y);
        sendMIDIrow(rowNum); //this is just to fix an issue with the LED being stuck on with the old WARBL and this Config Tool version.
    }
    else sendToWARBL(MIDI_CC_106, y);
    if (rowNum < 3) {
		sendMomentary(rowNum);
    }
}

function sendMIDIrow(MIDIrowNum) {
    blink(1);
    updateCells();
    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + MIDIrowNum);
    var y = (112) + parseFloat(document.getElementById("MIDIrow" + MIDIrowNum).value);

    sendToWARBL(MIDI_CC_102, y);
    //sendChannel(MIDIrowNum);
    //sendByte2(MIDIrowNum);
    //sendByte3(MIDIrowNum);

    if (MIDIrowNum < 3) {
        sendMomentary(MIDIrowNum);
    }

}

function sendChannel(rowNum) {
    blink(1);
    MIDIvalueChange();
    var y = parseFloat(document.getElementById("channel" + (rowNum)).value);
    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + rowNum);
    sendToWARBL(MIDI_CC_106, y);
}

function sendByte2(rowNum) {
    blink(1);
    MIDIvalueChange();
    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + rowNum);
    var y = parseFloat(document.getElementById("byte2_" + (rowNum)).value);
    sendToWARBL(MIDI_CC_107, y);
}

function sendByte3(rowNum) {
    blink(1);
    MIDIvalueChange();
    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + rowNum);
    var y = parseFloat(document.getElementById("byte3_" + (rowNum)).value);
    sendToWARBL(MIDI_CC_108, y);
}

function sendMomentary(rowNum) { //send momentary
    blink(1);
    updateCells();
    var y = document.getElementById("checkbox" + rowNum).checked

    sendToWARBL(MIDI_CC_102, MIDI_GESTURE_START + rowNum);
    if (y == false) {
        sendToWARBL(MIDI_CC_102, MIDI_MOMENTARY_OFF);
    }
    if (y == true) {
        sendToWARBL(MIDI_CC_102, MIDI_MOMENTARY_ON);
    }
}

//switches

function sendVented(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START);
    sendToWARBL(MIDI_CC_105, selection);
    updateSelected();
}

function sendBagless(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +1);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendSecret(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +2);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendInvert(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +3);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendCustom(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    updateCustom();
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +4);
    sendToWARBL(MIDI_CC_105, selection);
}


function sendExpression(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    
    sendToWARBL(MIDI_CC_104, MIDI_ED_VARS_START);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendRawPressure(selection) {
    selection = +selection; //convert true/false to 1/0
    updateCustom();
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SEND_PRESSURE);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendVelocity(selection) {
    selection = +selection; //convert true/false to 1/0
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +5);
    sendToWARBL(MIDI_CC_105, selection);
    updateCells();
}

function sendAftertouch(selection, polyselection) {
    selection = +selection; //convert true/false to 1/0
    var val = selection | ((+polyselection) << 1);
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +6);
    sendToWARBL(MIDI_CC_105, val);
}

function sendForceVelocity(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +7);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendHack1(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +8);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendHack2(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +9);
    sendToWARBL(MIDI_CC_105, selection);
}

function updateExpressionCenterPressureLabel()
{
    var slider = document.getElementById('exprfixedcenterslider');
    var element = document.getElementById('exprfixedcenterslider-value');
    var min = parseFloat(slider.value * 0.24).toFixed(1);
    element.innerHTML = min;
}

function updateExpressionCurveLabel(theslider, thelabel)
{
    var val = parseFloat(theslider.value - 64);
    if (val == 0) {
        thelabel.innerHTML = "Linear";
    } else if (val >= 0) {
        thelabel.innerHTML = ((3*val/63.0)+1).toFixed(1);
    }
    else {
        thelabel.innerHTML = (0.75*(64+val)/64.0 + 0.25).toFixed(3);
    }    
}

function updateExpressionCurveLabels()
{
    var lowslider = document.getElementById('exprcurvelowslider');
    var lowelement = document.getElementById('exprcurvelowslider-value');
    updateExpressionCurveLabel(lowslider, lowelement);
    
    var highslider = document.getElementById('exprcurvehighslider');
    var highelement = document.getElementById('exprcurvehighslider-value');
    updateExpressionCurveLabel(highslider, highelement);
}


function updateExpressionSliderEnableState()
{
    var overidden = document.getElementById("overrideExprCheck").checked;
    var overblow = document.getElementById("sensorradio1").checked;

    document.getElementById("expressionDepth").disabled = overidden || overblow;
    document.getElementById("exprfixedcenterslider").disabled = overidden || overblow;

    var dispval = version < 4.3 ? "none" : "block";
    document.getElementById("exprfixedcenterslider").style.display = dispval;
    document.getElementById("exprfixedcenterslider-value").style.display = dispval;
    document.getElementById("exprcenterpresslabel").style.display = dispval;

    document.getElementById("overrideExprCheck").disabled = overblow;
}

function curveValToExponent(val) {
    // takes 0 -> 127 and returns curve exponent
    // which is between 0.25 -> 0 -> 4.0
    var retval = 1.0;
    val -= 64;
    if (val >= 0) {
        retval = ((3.0*val/63.0) + 1.0);
    }
    else {
        retval = (0.75*(64.0+val)/64.0 + 0.25);
    }
    return retval;
}

function calculateOutExprBendFromInput(sensorValue)
{
    let overblow = document.getElementById("sensorradio1").checked;
    let overidden = document.getElementById("overrideExprCheck").checked;
    
    if (!overidden || overblow) {
        // do nothing!
        return 0.0;
    }

    // jlc
    // this mirrors the calculation on the warbl to give us just the expression bend component
    // not great, but better than adding a dedicated debug midi out message stream for this
    let lowPressureMin = parseInt(exprlowslider.noUiSlider.get()[0]) * 9 + 100;
    let lowPressureMax = parseInt(exprlowslider.noUiSlider.get()[1]) * 9 + 100;
    let highPressureMin = parseInt(exprhighslider.noUiSlider.get()[0]) * 9 + 100;
    let highPressureMax = parseInt(exprhighslider.noUiSlider.get()[1]) * 9 + 100;
    var centsLowOffset = 2 * (parseInt(exprlowbendslider.noUiSlider.get()[0]) - 64);
    var centsHighOffset = 2 * (parseInt(exprhighbendslider.noUiSlider.get()[0]) - 64);
    var lowCurveExp = curveValToExponent(parseInt(document.getElementById("exprcurvelowslider").value));
    var highCurveExp = curveValToExponent(parseInt(document.getElementById("exprcurvehighslider").value));
    let doClamp = document.getElementById("clampExprCheck").checked;
    

    var centsOffset = 0.0;
    var ratio = 0.0;
    let lowRangeSpan = Math.max(lowPressureMax - lowPressureMin, 1);
    let highRangeSpan = Math.max(highPressureMax - highPressureMin, 1);
    if (sensorValue <= lowPressureMax) {  // Pressure is in the lower range
        ratio = Math.min(((lowPressureMax - sensorValue)) / (lowRangeSpan), 1.0);
        if (lowCurveExp != 1.0) {
            ratio = Math.pow(ratio, lowCurveExp);
        }
        centsOffset = centsLowOffset * ratio;
    } else if (sensorValue >= highPressureMin) {  // Pressure is in the higher range
        ratio = Math.max(((sensorValue - highPressureMin)) / (highRangeSpan), 0.0);
        if (highCurveExp != 1.0) {
            ratio = Math.pow(ratio, highCurveExp);
        }
        if (doClamp) {
            ratio = Math.min(ratio, 1.0);
        }
        centsOffset = centsHighOffset * ratio;
    }
    else {
        // in the stable range
        centsOffset = 0;
    }
    
    // console.log("Cents out: " + centsOffset.toFixed(0) + " lm : " + lowPressureMin + " lx: " + lowPressureMax + " ratio: " + ratio + " sensorVal: " + sensorValue.toFixed(0));
    return centsOffset;
}

function sendOverride(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +10);
    sendToWARBL(MIDI_CC_105, selection);
    
    document.getElementById("overrideExprCheck").checked = selection;
    document.getElementById("checkbox16").checked = selection;
    
    updateExpressionSliderEnableState();
}

function sendBoth(selection) {
    selection = +selection;
    blink(1);
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +11);
    sendToWARBL(MIDI_CC_105, selection);
}

function sendR4flatten(selection) {
    selection = +selection;
    blink(1);
    updateCustom();
    sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START +12);
    sendToWARBL(MIDI_CC_105, selection);
}

function updateDoubleClick() {
	if (document.getElementById("cbDoubleClick").checked) {
		document.getElementById("gestureLabel0").innerHTML = "Double-click 1";
		document.getElementById("gestureLabel1").innerHTML = "Double-click 2";
		document.getElementById("gestureLabel2").innerHTML = "Double-click 3";
	} else {
		document.getElementById("gestureLabel0").innerHTML = "Click 1";
		document.getElementById("gestureLabel1").innerHTML = "Click 2";
		document.getElementById("gestureLabel2").innerHTML = "Click 3";
	}
}
function sendDoubleClick(selection) {
	updateDoubleClick();
	selection = +selection; 
	blink(1);
	sendToWARBL(MIDI_CC_104, MIDI_SWITCHES_VARS_START + 13);
	sendToWARBL(MIDI_CC_105, selection);
}

//end switches


function calibrateIMU() {
    blink(1);
    sendToWARBL(MIDI_CC_106, MIDI_CALIB_IMU);
}

function centerYaw() {
    blink(1);
    sendToWARBL(MIDI_CC_106, MIDI_CENTER_YAW);
}

function resetPitchExpressionOverride() {
    blink(1);
    sendToWARBL(MIDI_CC_106, MIDI_RESET_PITCH_EXPRESSION);

    setTimeout(function () {
        //console.log("refreshing UI");
        sendToWARBL(MIDI_CC_102, MIDI_ENTER_COMM_MODE);
        
        setTimeout(function () {
            mapPressure();
            overRideExpression();
        }, 500);
        
    }, 500);
}


function WARBL2Radio(selection) {
    blink(1);
    selection = parseFloat(selection);
    sendToWARBL(MIDI_CC_106, MIDI_WARBL2_SETTINGS_START);
    sendToWARBL(MIDI_CC_119, selection);
}


function sendHost(selection) {
    blink(1);

    selection = +selection; //convert true/false to 1/0
    sendToWARBL(MIDI_CC_106, MIDI_WARBL2_SETTINGS_START +1);
    sendToWARBL(MIDI_CC_119, selection);
}

function sendPoweroff(selection) {
    selection = parseFloat(selection);
    blink(1);
    sendToWARBL(MIDI_CC_106, MIDI_WARBL2_SETTINGS_START +2);
    sendToWARBL(MIDI_CC_119, selection);
}



function saveAsDefaults() {
    modalclose(2);
    blink(3);
    sendToWARBL(MIDI_CC_102, MIDI_SAVE_AS_DEFAULTS_CURRENT);
}

function saveCalibAsFactoryDefault() { // Only used for "factory calibtration"
	modalclose(2);
	blink(3);
	sendToWARBL(MIDI_CC_106, MIDI_SAVE_CALIB_AS_FACTORY);}

function saveAsDefaultsForAll() {
    modalclose(3);
    blink(3);
    sendToWARBL(MIDI_CC_102, MIDI_SAVE_AS_DEFAULTS_ALL);
}

function restoreAll() {
    modalclose(4);
    blink(3);
    sendToWARBL(MIDI_CC_102, MIDI_RESTORE_FACTORY);
    communicationMode = 0;
    if (version > 1.9 && version < 4.0) { //WARBL will restart, so try to reconnect to it.
        setTimeout(connect, 3000);
    }
}

function autoCalibrateBell() {
    if (communicationMode) {
        LEDon();
    }
    setTimeout(LEDoff, 5000);
    sendToWARBL(MIDI_CC_106, MIDI_CALIB_BELL_SENSOR);
}

function autoCalibrate() {
    if (communicationMode) {
        LEDon();
    }
    setTimeout(LEDoff, 10000);
    sendToWARBL(MIDI_CC_102, MIDI_START_CALIB);
}

function frequencyFromNoteNumber(note) {
    return 440 * Math.pow(2, (note - 69) / 12);
}

function modal(modalId) {
    document.getElementById("open-modal" + modalId).classList.add('modal-window-open');
    if (modalId == 18) { clearConsole(); }
    if (modalId == 25) {
        document.getElementById("sending").innerHTML = "Sending...";
        document.getElementById("WARBL2CustomSuccessOkay").style.display = "none";
    }
}

function modalclose(modalId) {
    document.getElementById("open-modal" + modalId).classList.remove('modal-window-open');
}

function blink(blinkNum) {
    if (communicationMode) {
        LEDon();
        setTimeout(LEDoff, 200);
        if (blinkNum > 1) { //blink twice
            setTimeout(LEDon, 400);
            setTimeout(LEDoff, 600);
        }
        if (blinkNum > 2) { // blink thrice
            setTimeout(LEDon, 800);
            setTimeout(LEDoff, 1000);
        }
        if (blinkNum > 3) { //blink four times
            setTimeout(LEDon, 1200);
            setTimeout(LEDoff, 1400);
        }
        if (blinkNum > 4) { //blink five times
            setTimeout(LEDon, 1600);
            setTimeout(LEDoff, 1800);
        }
    }
}

function checkMax(field) { //disable the sensor value field if it has been set to MAX
    var x = document.getElementById("v" + field).textContent;
    if (x == "MAX") {
        document.getElementById(field * 2).disabled = true;
    }
    if (x != "MAX") {
        document.getElementById(field * 2).disabled = false;
    }
}

function isEven(n) {
    return n % 2 == 0;
}

function isOdd(n) {
    return Math.abs(n % 2) == 1;
}

function LEDoff() {
    //console.log("off");
    elements = document.getElementsByClassName("leddot");
    for (var i = 0; i < elements.length; i++) {
        elements[i].style.visibility = "hidden";
    }
}

function LEDon() {
    elements = document.getElementsByClassName("leddot");
    for (var i = 0; i < elements.length; i++) {
        elements[i].style.visibility = "visible";
    }
}


function toggleOn() {
    volume = +!volume;
    if (volume == 0) {
		document.getElementById("volumeOff").style.display = "block";
		document.getElementById("volumeOn").style.display = "none";
        noteOff(currentNote);
    }
    else{
		document.getElementById("volumeOff").style.display = "none";
		document.getElementById("volumeOn").style.display = "block";
    }
}


function logKeys() {
    var s = 'Keys';
    for (var i = 0; i < midiNotes.length; i++) {
        s = s + ' ' + midiNotes[i].pitch;
    }
}

function noteOn(pitch) {
    noteOff(pitch);
    currentNote = pitch;
    if (volume) {
        for (var i = 0; i < _tone_0650_SBLive_sf2.zones.length; i++) {
            _tone_0650_SBLive_sf2.zones[i].ahdsr = false;
        }
        var envelope = player.queueWaveTable(audioContext, audioContext.destination, _tone_0650_SBLive_sf2, 0, pitch, 999, 0.5, true);
        var note = {
            pitch: pitch,
            envelope: envelope
        };
        midiNotes.push(note);
    }
}

function noteOff(pitch) {
    for (var i = 0; i < midiNotes.length; i++) {
        if (midiNotes[i].pitch == pitch) {
            if (midiNotes[i].envelope) {
                midiNotes[i].envelope.cancel();
            }
            midiNotes.splice(i, 1);
            return;
        }
    }
}

var output18 = document.getElementById("demo18");
var slideLimitSlider = document.getElementById('slidelimit');
slideLimitSlider.addEventListener('input', sliderLimitChange);
function sliderLimitChange() {
    output18.innerHTML = slideLimitSlider.value;
}


var output14 = document.getElementById("demo14");
var depthSlider = document.getElementById('depth');
depthSlider.addEventListener('input', slider14Change);
function slider14Change() {
    output14.innerHTML = depthSlider.value;
}

var output15 = document.getElementById("demo15");
var shakeDepthSlider = document.getElementById('shakeDepth');
shakeDepthSlider.addEventListener('input', slider15Change);
function slider15Change() {
    output15.innerHTML = shakeDepthSlider.value;
}

var output16 = document.getElementById("demo16");
var autoCenterYawSlider = document.getElementById('autoCenterYawInterval');
autoCenterYawSlider.addEventListener('input', slider16Change);
function slider16Change() {
    output16.innerHTML = autoCenterYawSlider.value / 4;
}

var output17 = document.getElementById("demo17");
var pitchRegisterSlider = document.getElementById('pitchRegisterNumber');
pitchRegisterSlider.addEventListener('input', slider17Change);
function slider17Change() {
    output17.innerHTML = pitchRegisterSlider.value;
}

var output1 = document.getElementById("demo1");
var jumpSlider1 = document.getElementById('jumpFactor1');
jumpSlider1.addEventListener('input', slider1Change);
function slider1Change() {
    output1.innerHTML = jumpSlider1.value;
}

var output2 = document.getElementById("demo2");
var jumpSlider2 = document.getElementById('jumpFactor2');
jumpSlider2.addEventListener('input', slider2Change);
function slider2Change() {
    output2.innerHTML = jumpSlider2.value;
}

var output3 = document.getElementById("demo3");
var jumpSlider3 = document.getElementById('jumpFactor3');
jumpSlider3.addEventListener('input', slider3Change);
function slider3Change() {
    output3.innerHTML = jumpSlider3.value;
}

var output4 = document.getElementById("demo4");
var jumpSlider4 = document.getElementById('jumpFactor4');
jumpSlider4.addEventListener('input', slider4Change);
function slider4Change() {
    output4.innerHTML = jumpSlider4.value;
}

var output5 = document.getElementById("demo5");
var jumpSlider5 = document.getElementById('jumpFactor5');
jumpSlider5.addEventListener('input', slider5Change);
function slider5Change() {
    output5.innerHTML = jumpSlider5.value;
}

var output6 = document.getElementById("demo6");
var jumpSlider6 = document.getElementById('jumpFactor6');
jumpSlider6.addEventListener('input', slider6Change);
function slider6Change() {
    output6.innerHTML = jumpSlider6.value;
}

var output7 = document.getElementById("demo7");
var jumpSlider7 = document.getElementById('jumpFactor7');
jumpSlider7.addEventListener('input', slider7Change);
function slider7Change() {
    output7.innerHTML = jumpSlider7.value;
}

var output8 = document.getElementById("demo8");
var jumpSlider8 = document.getElementById('jumpFactor8');
jumpSlider8.addEventListener('input', slider8Change);
function slider8Change() {
    output8.innerHTML = jumpSlider8.value;
}

var output9 = document.getElementById("demo9");
var jumpSlider9 = document.getElementById('jumpFactor9');
jumpSlider9.addEventListener('input', slider9Change);
function slider9Change() {
    output9.innerHTML = jumpSlider9.value;
}

var output10 = document.getElementById("demo10");
var jumpSlider10 = document.getElementById('jumpFactor10');
jumpSlider10.addEventListener('input', slider10Change);
function slider10Change() {
    output10.innerHTML = jumpSlider10.value;
}

var output10b = document.getElementById("demo10b");
var jumpSlider10b = document.getElementById('jumpFactor10b');
jumpSlider10b.addEventListener('input', slider10bChange);
function slider10bChange() {
    output10b.innerHTML = jumpSlider10b.value;
}

var output11 = document.getElementById("demo11");
var jumpSlider11 = document.getElementById('jumpFactor11');
jumpSlider11.addEventListener('input', slider11Change);
function slider11Change() {
    output11.innerHTML = jumpSlider11.value;
}

var output12 = document.getElementById("demo12");
var jumpSlider12 = document.getElementById('jumpFactor12');
jumpSlider12.addEventListener('input', slider12Change);
function slider12Change() {
    output12.innerHTML = jumpSlider12.value;
}

var outputPoweroff = document.getElementById("poweroffValue");
var poweroffSlider = document.getElementById('poweroffSlider');
poweroffSlider.addEventListener('input', poweroffSliderChange);
function poweroffSliderChange() {
    outputPoweroff.innerHTML = poweroffSlider.value;
}



function clearConsole() { //clear MIDI console
    document.getElementById("console").innerHTML = "";
    consoleEntries = 0;
}



//sets up initial values for selects/fields/radios and constantly keeps the proper options enabled/disabled
function updateCells() {

    if (version > 1.6) {
        var p = document.getElementById("checkbox10").checked;
        if (p == true) {
            document.getElementById("checkbox12").disabled = true;
            document.getElementById("switch12").style.cursor = "default";
        } else {
            document.getElementById("checkbox12").disabled = false;
            document.getElementById("switch12").style.cursor = "pointer";
        }
    }

    var q = document.getElementById("checkbox0").checked;
    if (q == true) {
        document.getElementById("row5").disabled = true;
    } else {
        document.getElementById("row5").disabled = false;
    }

    var r = document.getElementById("checkbox1").checked;
    if (r == true) {
        document.getElementById("row6").disabled = true;
    } else {
        document.getElementById("row6").disabled = false;
    }

    if (q == true || r == true) {
        document.getElementById("row3").disabled = true;
    } else {
        document.getElementById("row3").disabled = false;
    }

    var s = document.getElementById("checkbox2").checked;
    if (s == true) {
        document.getElementById("row7").disabled = true;
    } else {
        document.getElementById("row7").disabled = false;
    }

    if (r == true || s == true) {
        document.getElementById("row4").disabled = true;
    } else {
        document.getElementById("row4").disabled = false;
    }
	


    for (var i = 0; i < numberOfGestures; i++) {
        var x = document.getElementById("row" + i).value;
        var t = document.getElementById("row" + i).disabled;

        if ((x != 1 || t == true)) {
            document.getElementById("MIDIrow" + i).disabled = true;
            document.getElementById("channel" + i).disabled = true;
            document.getElementById("byte2_" + i).disabled = true;
            document.getElementById("byte3_" + i).disabled = true;
        } else {
            document.getElementById("MIDIrow" + i).disabled = false;
            document.getElementById("channel" + i).disabled = false;
            document.getElementById("byte2_" + i).disabled = false;
            document.getElementById("byte3_" + i).disabled = false;
        }

        var y = document.getElementById("MIDIrow" + i).value;
        if (y == 2) {
            document.getElementById("byte3_" + i).disabled = true;
        }
        if (y > 2) {
            document.getElementById("byte2_" + i).disabled = true;
            document.getElementById("byte3_" + i).disabled = true;
        }

        var z = document.getElementById("row" + i).value;
		
        if (((y == 0 || (version > 4.1 && y == 1)) && i < 3 && x == 1) || ((version > 1.4 || version == "Unknown") && i < 3 && (z == 5 || z == 6 || z == 10 || z == 11))) {
            document.getElementById("checkbox" + i).disabled = false;
            document.getElementById("switch" + i).style.cursor = "pointer";
        }

        if (((((y != 0 && version <= 4.1) || (version > 4.1 && y > 1)) && i < 3) || (i < 3 && (x == 0 || x > 1))) && !((version > 1.4 || version == "Unknown") && i < 3 && (z == 5 || z == 6 || z == 10 || z == 11))) {
            document.getElementById("checkbox" + i).disabled = true;
            document.getElementById("switch" + i).style.cursor = "default";
            document.getElementById("checkbox" + i).checked = false;
        }

        var q = document.getElementById("checkbox0").checked;
        if (q == true) {
            document.getElementById("row5").disabled = true;
        } else {
            document.getElementById("row5").disabled = false;
        }

        var r = document.getElementById("checkbox1").checked;
        if (r == true) {
            document.getElementById("row6").disabled = true;
        } else {
            document.getElementById("row6").disabled = false;
        }

        if (q == true || r == true) {
            document.getElementById("row3").disabled = true;
        } else {
            document.getElementById("row3").disabled = false;
        }

        var s = document.getElementById("checkbox2").checked;
        if (s == true) {
            document.getElementById("row7").disabled = true;
        } else {
            document.getElementById("row7").disabled = false;
        }

        if (r == true || s == true) {
            document.getElementById("row4").disabled = true;
        } else {
            document.getElementById("row4").disabled = false;
        }

    }

    var z = $("#fingeringSelect0 option:selected").text();
    document.getElementById("fingeringLabel1").innerHTML = z;

    var z = $("#fingeringSelect1 option:selected").text();
    document.getElementById("fingeringLabel2").innerHTML = z;

    var z = $("#fingeringSelect2 option:selected").text();
    document.getElementById("fingeringLabel3").innerHTML = z;

}

function MIDIvalueChange() {
    for (var i = 0; i < numberOfGestures; i++) {
        var x = document.getElementById("channel" + i).value;
        if (((x < 0 || x > 16 || isNaN(x)) && !document.getElementById("channel" + i).disabled)) {
            alert("Value must be 1-16.");
            document.getElementById("channel" + i).value = null;
        }
    }
    for (var j = 0; j < numberOfGestures; j++) {
        var y = document.getElementById("channel" + j).value;
        var x = document.getElementById("byte2_" + j).value;
        if (x < 0 || x > 127 || isNaN(x)) {
            alert("Value must be 0-127.");
            document.getElementById("byte2_" + j).value = null;
        }
        if (y == 7 && x > 101 && x < 120) {
            alert("This CC range on channel 7 is reserved for the Configuration Tool.");
            document.getElementById("byte2_" + j).value = null;

        }
    }

    for (var k = 0; k < numberOfGestures; k++) {
        var x = document.getElementById("byte3_" + k).value;
        if (x < 0 || x > 127 || isNaN(x)) {
            alert("Value must be 0-127.");
            document.getElementById("byte3_" + k).value = null;
        }
    }

}

//
// WebAudio related functionality
//
var AudioContextFunc = window.AudioContext || window.webkitAudioContext;

var audioContext = new AudioContextFunc();

var player = new WebAudioFontPlayer();

player.loader.decodeAfterLoading(audioContext, '_tone_0650_SBLive_sf2');

//
// Toggle between adding and removing the "responsive" class to topnav when the user clicks on the icon
//
function topNavFunction() {

    var x = document.getElementById("myTopnav");

    if (x.className === "topnav") {
        x.className += " responsive";
    } else {
        x.className = "topnav";
    }

}

//
// Preset import/export features
//

// 
// Synchronous sleep (returns Promise for async/await use)
//
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

//
// Import a WARBL preset from a file
//


function importPreset(context) {

    //console.log("importPreset");

    //console.log("current tab = "+instrument);

    // Put up error message if WARBL is not connected
    if (!WARBLout) {

        document.getElementById("modal14-title").innerHTML = "Preset import only allowed when WARBL is connected";

        document.getElementById("modal14-ok").style.opacity = 1.0;

        modal(14);

        return;

    }

    // For versioning preset files
    var maxSupportedPresetVersion = 1;

    // How long to wait in msec between sending commands
    var delay = 10;

    //debugger;

    // Make sure there is a file selected
    if (context.files && (context.files.length != 0)) {

        // Show the import modal
        var fname = context.files[0].name;
        fname = fname.replace(".warbl", "");
        fname = fname.replace(".WARBL", "");

        document.getElementById("modal14-title").innerHTML = "Importing: " + context.files[0].name;

        // Disable the OK button until after the import is complete
        document.getElementById("modal14-ok").style.opacity = 0.001;

        modal(14);

        var reader = new FileReader();

        reader.onload = async function () {

            //debugger;

            // Parse the file
            try {

                var theImportObject = JSON.parse(reader.result);

            } catch (e) {

                document.getElementById("modal14-title").innerHTML = "Error: File is not a WARBL Preset";
                return;
				document.getElementById("modal14-ok").style.opacity = 1;
            }

            // Sanity check the import file

            if (!theImportObject) {

                document.getElementById("modal14-title").innerHTML = "Error: File is not a WARBL Preset";
                return;
				document.getElementById("modal14-ok").style.opacity = 1;
            }

            if ((!theImportObject.signature) || (theImportObject.signature != "WARBL")) {

                document.getElementById("modal14-title").innerHTML = "Error: File is not a WARBL Preset";
                return;
				document.getElementById("modal14-ok").style.opacity = 1;
            }

            if (!theImportObject.version) {

                document.getElementById("modal14-title").innerHTML = "Error: File is not a WARBL Preset";
                return;
				document.getElementById("modal14-ok").style.opacity = 1;
            }

            if (theImportObject.version > maxSupportedPresetVersion) {
                document.getElementById("modal14-title").innerHTML = "Error: WARBL Preset Version " + theImportObject.version + " not supported by this version of the configuration tool";
				document.getElementById("modal14-ok").style.opacity = 1;
                return;

            }
			
			if ((theImportObject.messages[0][2] > 40 && version < 4.0) || (theImportObject.messages[0][2] < 40 && version > 4.0)) {

                document.getElementById("modal14-title").innerHTML = "Sorry, it is not possible to import settings from the original WARBL to WARBL2 or vice versa.";
				document.getElementById("modal14-ok").style.opacity = 1;
                return;

            }
			
			


            //debugger;

            // Send the data to the WARBL
            var nMessages = theImportObject.messages.length;

            var i;
            var byte0, byte1, byte2;

            // Send the firmware version
            byte0 = theImportObject.messages[0][0];
            byte1 = theImportObject.messages[0][1];
            byte2 = theImportObject.messages[0][2];
            sendToWARBL(byte1, byte2);

            // Synchronous sleep to allow command processing
            await sleep(delay);

            //
            // Determine the current instrument that was set at export
            //
            var exportedInstrument = theImportObject.messages[10][2] - 60;

            //
            // Determine the current target instrument tab
            //
            var targetInstrument = 30 + instrument;

            // console.log("exportedInstrument = "+exportedInstrument);
            // console.log("targetInstrument = "+(targetInstrument-30));

            //
            // Determine the current target key shift
            //
            var targetKey = 111 + instrument;

            // send the fingering and key shift for the current tab only
            switch (exportedInstrument) {

                case 0:

                    byte0 = theImportObject.messages[1][0];
                    byte1 = theImportObject.messages[1][2];
                    byte2 = targetInstrument;

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[2][0];
                    byte1 = theImportObject.messages[2][1];
                    byte2 = theImportObject.messages[2][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[3][0];
                    byte1 = targetKey;
                    byte2 = theImportObject.messages[3][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    break;

                case 1:

                    byte0 = theImportObject.messages[4][0];
                    byte1 = theImportObject.messages[4][1];
                    byte2 = targetInstrument;

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[5][0];
                    byte1 = theImportObject.messages[5][1];
                    byte2 = theImportObject.messages[5][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[6][0];
                    byte1 = targetKey;
                    byte2 = theImportObject.messages[6][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    break;

                case 2:

                    byte0 = theImportObject.messages[7][0];
                    byte1 = theImportObject.messages[7][1];
                    byte2 = targetInstrument;

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[8][0];
                    byte1 = theImportObject.messages[8][1];
                    byte2 = theImportObject.messages[8][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    byte0 = theImportObject.messages[9][0];
                    byte1 = targetKey;
                    byte2 = theImportObject.messages[9][2];

                    sendToWARBL(byte1, byte2);

                    // Synchronous sleep to allow command processing
                    await sleep(delay);

                    break;

            }

            // Skip command 10 - Selected instrument at save time

            // Skip command 11 - Sets default

            // Send the rest of the data

            for (i = 12; i < nMessages; ++i) {

                byte0 = theImportObject.messages[i][0];
                byte1 = theImportObject.messages[i][1];
                byte2 = theImportObject.messages[i][2];

                //console.log("Sending Message #"+i+":"+byte0+" "+byte1+" "+byte2);

                sendToWARBL(byte1, byte2);

                // Synchronous sleep to allow command processing
                await sleep(delay);

            }

            // Refresh the UI after a short delay by putting the device back in communications mode
            setTimeout(function () {

                //console.log("refreshing UI");

                sendToWARBL(MIDI_CC_102, MIDI_ENTER_COMM_MODE);

                // Show the import complete modal
                document.getElementById("modal14-title").innerHTML = "Preset Import Complete!";

                // Show the OK button until after the import is complete
                document.getElementById("modal14-ok").style.opacity = 1.0;

                // Clear the file input selector after a delay
                setTimeout(function () {

                    $('#importPreset').val('');

                }, 1000);

            }, 500);

        }

        reader.readAsText(context.files[0]);

    }

}

//
// Export the current WARBL preset to a file
//
function exportPreset() {

    //console.log("exportPreset");

    // Put up error message if WARBL is not connected
    if (!WARBLout) {

        document.getElementById("modal14-title").innerHTML = "Preset export only allowed when WARBL is connected";

        document.getElementById("modal14-ok").style.opacity = 1.0;

        modal(14);

        return;

    }

    var nMessages = 0;

    // Initialize the export structure
    var theExportObject = {
        signature: "WARBL",  // Used to sanity check imports
        version: 1,			// In case we need to version the export structure
        messages: []			// Preset messages
    };

    // 
    // Download the exported preset to a file
    //
    function downloadPreset(content, filename, contentType) {

        const a = document.createElement('a');

        const file = new Blob([content], { type: contentType });

        a.href = URL.createObjectURL(file);
        a.download = filename;
        a.click();

        URL.revokeObjectURL(a.href);

        // Tell the user that the export is complete
        document.getElementById("modal14-title").innerHTML = "Preset Export Complete!";

        // Make sure the OK button is showing
        document.getElementById("modal14-ok").style.opacity = 1.0;

        modal(14);

    };

    //
    // Export message proxy
    //
    function exportMessageHandler(byte0, byte1, byte2) {

        //console.log("Message #"+nMessages+":"+byte0+" "+byte1+" "+byte2);

        // Add the message to the message array
        theExportObject.messages.push([byte0, byte1, byte2]);

        nMessages++;

        if (nMessages == 135) {

            //debugger;

            //console.log("exportMessageHandler: All messages received");

            // Stringify the messsages array
            var theExportObjectJSON = JSON.stringify(theExportObject);

            // Clear the global export proxy
            gExportProxy = null;

            // Do the preset file export
            downloadPreset(theExportObjectJSON, "WARBL_Preset.warbl", "text/plain");

        }
    }

    // Setup the global export message proxy
    gExportProxy = exportMessageHandler;

    // Tell WARBL to enter communications mode
    // Received bytes will be forwarded to the export message handler instead
    sendToWARBL(MIDI_CC_102, MIDI_ENTER_COMM_MODE);

}

//
// Capture the incoming pressure
//
function capturePressure(val) {

    //console.log("capturePressure: "+val);

    gCurrentPressure = val;

}

//
// Graph the incoming pressure
//
function graphPressure() {

    //console.log("graphPressure: "+gCurrentPressure);
    //debugger;

    // Drop the last element of the array
    gPressureReadings.pop();

    // Add the new value to the front of the array
    gPressureReadings.unshift(gCurrentPressure);

    //debugger;

    // Clear the graph
    $("#d3PressureGraph").empty();

    // set the dimensions and margins of the graph
    var margin = { top: 10, right: 10, bottom: 100, left: 50 },
        width = 390,
        height = 320 - margin.top - margin.bottom;

    // append the svg object to the body of the page
    var svg = d3.select("#d3PressureGraph")
        .append("svg")
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        // translate this svg element to leave some margin.
        .append("g")
        .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    // X scale and Axis
    var x = d3.scaleLinear()
        .domain([0, width])       // This is the min and the max of the data
        .range([0, width]);

    svg.append('g')
        .attr("transform", "translate(0," + height + ")")
        .call(d3.axisBottom(x).tickSize(0).tickFormat(""));

    // Y scale and Axis
    var y = d3.scaleLinear()
        .domain([0, gMaxPressure])  // This is the min and the max of the data: 0 to gMaxPressure
        .range([height, 0]);

    svg.append('g')
        .call(d3.axisLeft(y));

    // text label for the x axis
    svg.append("text")
        .attr("y", (height + 4))
        .attr("x", (width / 2) - (margin.left / 2) + 10)
        .attr("dy", "1em")
        .style("text-anchor", "middle")
        .style("fill", "white")
        .text("Time");

    // text label for the y axis
    svg.append("text")
        .attr("transform", "rotate(-90)")
        .attr("y", 0 - (margin.left - 5))
        .attr("x", 0 - (height / 2))
        .attr("dy", "1em")
        .style("text-anchor", "middle")
        .style("fill", "white")
        .text("Inches of H₂O");

    // X scale will use the index of our data
    var xScale = d3.scaleLinear()
        .domain([0, gNPressureReadings - 1]) // input
        .range([0, width]); // output

    // Y scale will use the pressure range from 0 - gMaxPressure 
    var yScale = d3.scaleLinear()
        .domain([0, gMaxPressure]) // input 
        .range([height, 0]); // output 

    // d3's line generator
    var line = d3.line()
        .x(function (d, i) { return xScale(i); }) // set the x values for the line generator
        .y(function (d) { return yScale(d.y); }) // set the y values for the line generator 
        .curve(d3.curveMonotoneX); // apply smoothing to the line

    // An array of objects of length N. Each object has key -> value pair, the key being "y" and the value is a pressure reading
    var dataset = d3.range(gNPressureReadings).map(function (d) { return { "y": gPressureReadings[d] } });

    // Append the path, bind the data, and call the line generator 
    svg.append("path")
        .datum(dataset) // Binds data to the line 
        .attr("class", "pressureline") // Assign a class for styling 
        .attr("d", line); // Calls the line generator 
}

//
// Start the pressure graph
//
function startPressureGraph() {

    //console.log("startPressureGraph");

    // Shunt the pressure messages to the graphing routine
    gPressureGraphEnabled = true;

    // Reset the pressure capture
    gCurrentPressure = 0.0;

    // Clear the pressure capture array
    var i;
    gPressureReadings = [];
    for (i = 0; i < gNPressureReadings; ++i) {
        gPressureReadings.push(0.0);
    }

    // Setup the graph redraw interval timer
    gGraphInterval = setInterval(graphPressure, 100);

}

//
// Stop the pressure graph
//
function stopPressureGraph() {

    //console.log("stopPressureGraph");

    // Stop shunting the pressure messages to the graphing routine
    gPressureGraphEnabled = false;

    // Stop the interval timer
    clearInterval(gGraphInterval);

    $("#d3PressureGraph").empty();

    return;
}