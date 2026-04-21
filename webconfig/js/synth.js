// Synth

// Fixed reset delay (ms)
// This resets expression to the max in case a user was sending expression and then stopped.
var RESET_DELAY_MS = 2000;

var audioContext = null;
var synth = null;
var currentInputId = null;
var synthReady = false;
var startAudioPromise = null;

// Pitch bend range
var pitchBendRangeSemitones = document.getElementById("synthBendRange").value;
if (localStorage.getItem("pitchBendRange") != null) {
	pitchBendRangeSemitones = localStorage.getItem("pitchBendRange");
	document.getElementById("synthBendRange").value = pitchBendRangeSemitones;
}

// Voice
var synthPresetNumber = 1;
if (localStorage.getItem("synthPreset") != null) {
	synthPresetNumber = localStorage.getItem("synthPreset");
	document.getElementById("presetSelect").value = synthPresetNumber;
}

// Reverb
var synthReverbValue = 20;
if (localStorage.getItem("synthReverb") != null) {
	synthReverbValue = localStorage.getItem("synthReverb");
	document.getElementById("reverbSlider").value = synthReverbValue;
}

// Transpose
var synthTransposeValue = 0;
if (localStorage.getItem("synthTranspose") != null) {
	synthTransposeValue = localStorage.getItem("synthTranspose");
	document.getElementById("synthKeySelect").value = synthTransposeValue;
}

// Per-channel state
var volumeValue = new Array(16);
var volumeTarget = new Array(16);
var volumeCurrent = new Array(16);
var ccSeenSinceNoteOn = new Array(16);
var volumeResetTimers = new Array(16);
var volumeSmootherTimer = null;

var activeNotesByChannel = new Array(16);
var lastPitchBendValue = new Array(16);
var lastChannelPressureValue = new Array(16);
var lastCCValue = new Array(16);

// MIDI burst-coalescing wrapper
var MIDI_FLUSH_MS = 8; // try 5 first; 8 is safer, 3 is snappier
var midiFlushTimer = null;

var pendingPitchBend = new Array(16);
var pendingChannelPressure = new Array(16);
var pendingCC = new Array(16);

for (var i = 0; i < 16; i++) {
	volumeValue[i] = 127;
	volumeTarget[i] = 127;
	volumeCurrent[i] = 127;
	ccSeenSinceNoteOn[i] = false;
	volumeResetTimers[i] = null;

	activeNotesByChannel[i] = {};
	lastPitchBendValue[i] = 8192;
	lastChannelPressureValue[i] = 0;
	lastCCValue[i] = {};

	pendingPitchBend[i] = null;
	pendingChannelPressure[i] = null;
	pendingCC[i] = {};
}

var statusEl = document.getElementById("audioStatus");

function setStatus(text) {
	if (statusEl) {
		statusEl.textContent = text;
	}
	console.log(text);
}

function saveTranspose(semitones) {
	synthTransposeValue = semitones;
	localStorage.setItem("synthTranspose", semitones);
	setGlobalTranspose(synthTransposeValue);
}

function setGlobalTranspose(semitones) {
	if (!synth) return;
	synth.setMasterParameter("transposition", Number(semitones) || 0);
}

function control(ch, num, val) {
	if (!synth) return;

	ch = ch & 0x0f;
	num = num & 0x7f;
	val = Math.max(0, Math.min(127, val | 0));

	if (typeof synth.controllerChange === "function") {
		synth.controllerChange(ch, num, val);
	} else if (typeof synth.controlChange === "function") {
		synth.controlChange(ch, num, val);
	}
}

function pb(ch, value14) {
	if (!synth) return;

	ch = ch & 0x0f;
	value14 = Math.max(0, Math.min(16383, value14 | 0));

	if (typeof synth.pitchWheel === "function") {
		synth.pitchWheel(ch, value14);
	}
}

function channelPressure(ch, value) {
	if (!synth) return;

	ch = ch & 0x0f;
	value = Math.max(0, Math.min(127, value | 0));

	if (typeof synth.channelPressure === "function") {
		synth.channelPressure(ch, value);
	} else if (typeof synth.channelAftertouch === "function") {
		synth.channelAftertouch(ch, value);
	} else if (typeof synth.aftertouch === "function") {
		synth.aftertouch(ch, value);
	}
}

function clearPendingMidiForChannel(ch) {
	ch = ch & 0x0f;
	pendingPitchBend[ch] = null;
	pendingChannelPressure[ch] = null;
	pendingCC[ch] = {};
}

function clearAllPendingMidi() {
	for (var ch = 0; ch < 16; ch++) {
		clearPendingMidiForChannel(ch);
	}

	if (midiFlushTimer !== null) {
		clearInterval(midiFlushTimer);
		midiFlushTimer = null;
	}
}

function hasAnyPendingMidi() {
	for (var ch = 0; ch < 16; ch++) {
		if (pendingPitchBend[ch] !== null || pendingChannelPressure[ch] !== null) {
			return true;
		}

		for (var ccNum in pendingCC[ch]) {
			if (Object.prototype.hasOwnProperty.call(pendingCC[ch], ccNum)) {
				return true;
			}
		}
	}

	return false;
}

function startMidiFlushTimer() {
	if (midiFlushTimer !== null) return;

	midiFlushTimer = setInterval(function () {
		if (!synthReady || !synth) return;

		for (var ch = 0; ch < 16; ch++) {
			if (pendingPitchBend[ch] !== null) {
				pb(ch, pendingPitchBend[ch]);
				pendingPitchBend[ch] = null;
			}

			var ccState = pendingCC[ch];
			for (var ccNum in ccState) {
				if (Object.prototype.hasOwnProperty.call(ccState, ccNum)) {
					control(ch, parseInt(ccNum, 10), ccState[ccNum]);
					delete ccState[ccNum];
				}
			}

			if (pendingChannelPressure[ch] !== null) {
				channelPressure(ch, pendingChannelPressure[ch]);
				pendingChannelPressure[ch] = null;
			}
		}

		if (!hasAnyPendingMidi()) {
			clearInterval(midiFlushTimer);
			midiFlushTimer = null;
		}
	}, MIDI_FLUSH_MS);
}

function queuePitchBend(ch, value14) {
	ch = ch & 0x0f;
	pendingPitchBend[ch] = value14;
	startMidiFlushTimer();
}

function queueChannelPressure(ch, value) {
	ch = ch & 0x0f;
	pendingChannelPressure[ch] = value;
	startMidiFlushTimer();
}

function queueControl(ch, num, val) {
	ch = ch & 0x0f;
	num = num & 0x7f;
	pendingCC[ch][num] = val;
	startMidiFlushTimer();
}

function flushPendingMidiForChannel(ch) {
	ch = ch & 0x0f;

	if (pendingPitchBend[ch] !== null) {
		pb(ch, pendingPitchBend[ch]);
		pendingPitchBend[ch] = null;
	}

	var ccState = pendingCC[ch];
	for (var ccNum in ccState) {
		if (Object.prototype.hasOwnProperty.call(ccState, ccNum)) {
			control(ch, parseInt(ccNum, 10), ccState[ccNum]);
			delete ccState[ccNum];
		}
	}

	if (pendingChannelPressure[ch] !== null) {
		channelPressure(ch, pendingChannelPressure[ch]);
		pendingChannelPressure[ch] = null;
	}
}

function hasActiveNotesOnChannel(ch) {
	return Object.keys(activeNotesByChannel[ch & 0x0f]).length > 0;
}

function applyStoredChannelState(ch) {
	ch = ch & 0x0f;

	// Flush queued real-time updates first so the freshest state wins.
	flushPendingMidiForChannel(ch);

	// Re-apply cached state in case nothing was queued.
	pb(ch, lastPitchBendValue[ch]);

	var ccState = lastCCValue[ch];
	for (var ccNum in ccState) {
		if (Object.prototype.hasOwnProperty.call(ccState, ccNum)) {
			control(ch, parseInt(ccNum, 10), ccState[ccNum]);
		}
	}

	channelPressure(ch, lastChannelPressureValue[ch]);
}

function synthNoteOn(ch, note, velocity) {
	if (!synth) return;

	ch = ch & 0x0f;
	note = note & 0x7f;

	if (synthPresetNumber == 2 && note < 55) { // If uilleann play drones in correct key
		note = 50;
	}
	if ((synthPresetNumber == 4 || synthPresetNumber == 5) && note < 55) { // If GHB or smallpipes play drones in correct key
		note = 46;
	}

	velocity = Math.max(0, Math.min(127, velocity | 0));

	if (velocity === 0) {
		synthNoteOff(ch, note);
		return;
	}

	// Re-apply cached/queued channel state before note-on so messages
	// received before the note still affect its start.
	applyStoredChannelState(ch);

	activeNotesByChannel[ch][note] = true;
	synth.noteOn(ch, note, velocity);

	scheduleVolumeResetAfterNoteOn(ch);
}

function synthNoteOff(ch, note) {
	if (!synth) return;

	ch = ch & 0x0f;
	note = note & 0x7f;

	if (synthPresetNumber == 2 && note < 55) { // If uilleann play drones in correct key
		note = 50;
	}
	if ((synthPresetNumber == 4 || synthPresetNumber == 5) && note < 55) { // If GHB or smallpipes play drones in correct key
		note = 46;
	}

	delete activeNotesByChannel[ch][note];
	synth.noteOff(ch, note);
}

function handlePitchBend(ch, value14) {
	ch = ch & 0x0f;
	value14 = Math.max(0, Math.min(16383, value14 | 0));

	lastPitchBendValue[ch] = value14;

	// Only apply to currently sounding notes on this channel.
	if (hasActiveNotesOnChannel(ch)) {
		queuePitchBend(ch, value14);
	}
}

function handleChannelPressure(ch, value) {
	ch = ch & 0x0f;
	value = Math.max(0, Math.min(127, value | 0));

	lastChannelPressureValue[ch] = value;

	// Channel 1 (external) acts globally for MPE-style expression.
	if (ch === 0) {
		for (var targetCh = 0; targetCh < 16; targetCh++) {
			lastChannelPressureValue[targetCh] = value;

			if (hasActiveNotesOnChannel(targetCh)) {
				queueChannelPressure(targetCh, value);
			}
		}
		return;
	}

	// Only apply immediately to sounding notes on this channel.
	if (hasActiveNotesOnChannel(ch)) {
		queueChannelPressure(ch, value);
	}
}

function handleControlChange(ch, num, val) {
	ch = ch & 0x0f;
	num = num & 0x7f;
	val = Math.max(0, Math.min(127, val | 0));

	var isExpressionCC = (num === 2 || num === 11 || num === 7);
	var isGlobalMPEExpression = (ch === 0 && isExpressionCC); // MIDI channel 1 externally

	// In MPE mode, expression on channel 1 acts globally.
	if (isGlobalMPEExpression) {
		for (var targetCh = 0; targetCh < 16; targetCh++) {
			lastCCValue[targetCh][num] = val;

			// Keep loudness mapping in sync on every channel.
			handleVolumeControl(targetCh, val);

			// Do not immediately spam the synth with expression bursts.
			// The volume smoother sends CC11 steadily.
		}
		return;
	}

	// Normal per-channel behavior for everything else.
	lastCCValue[ch][num] = val;

	if (isExpressionCC) {
		handleVolumeControl(ch, val);

		// Do not immediately send expression bursts.
		// Let the smoother drive CC11 output at a stable rate.
		return;
	}

	if (hasActiveNotesOnChannel(ch)) {
		queueControl(ch, num, val);
	}
}

function connectSynthOutput() {
	if (!synth || !audioContext) return;

	if (typeof synth.connect === "function") {
		synth.connect(audioContext.destination);
	} else if (synth.output && typeof synth.output.connect === "function") {
		synth.output.connect(audioContext.destination);
	} else if (synth.worklet && typeof synth.worklet.connect === "function") {
		synth.worklet.connect(audioContext.destination);
	}
}

function disconnectSynthOutput() {
	if (!synth) return;

	try {
		if (typeof synth.disconnect === "function") {
			synth.disconnect();
		} else if (synth.output && typeof synth.output.disconnect === "function") {
			synth.output.disconnect();
		} else if (synth.worklet && typeof synth.worklet.disconnect === "function") {
			synth.worklet.disconnect();
		}
	} catch (e) {
		console.warn("disconnectSynthOutput error:", e);
	}
}

function stopAllSynthNotes() {
	clearAllPendingMidi();
	clearAllVolumeResetTimers();

	if (!synth) return;

	for (var ch = 0; ch < 16; ch++) {
		try {
			control(ch, 64, 0);   // sustain off
			control(ch, 123, 0);  // all notes off
			control(ch, 120, 0);  // all sound off

			if (typeof synth.allNotesOff === "function") {
				synth.allNotesOff(ch);
			}
		} catch (e) {
			console.warn("stopAllSynthNotes error on channel", ch, e);
		}

		activeNotesByChannel[ch] = {};
	}
}

function setPitchBendRange(semitones, cents) {
	if (!synth) return;

	semitones = Math.max(0, Math.min(127, semitones | 0));
	cents = (typeof cents === "number") ? Math.max(0, Math.min(127, cents | 0)) : 0;

	for (var ch = 0; ch < 16; ch++) {
		control(ch, 101, 0);
		control(ch, 100, 0);
		control(ch, 6, semitones);
		control(ch, 38, cents);
		control(ch, 101, 127);
		control(ch, 100, 127);
	}
}

function setChannelVolume(ch, value) {
	ch = ch & 0x0f;
	value = Math.max(0, Math.min(127, value | 0));

	volumeValue[ch] = value;
	volumeTarget[ch] = value;

	// Cache the expression state so it can be re-applied on the next note.
	lastCCValue[ch][11] = value;
}

function startVolumeSmoother() {
	if (volumeSmootherTimer !== null) return;

	volumeSmootherTimer = setInterval(function () {
		if (!synthReady || !synth) return;

		for (var ch = 0; ch < 16; ch++) {
			var target = volumeTarget[ch];
			var current = volumeCurrent[ch];

			// Smoothing coefficient:
			// lower = smoother, higher = more responsive
			current += (target - current) * 0.28;

			if (Math.abs(target - current) < 0.35) {
				current = target;
			}

			volumeCurrent[ch] = current;

			// Smooth expression only
			control(ch, 11, Math.round(current));
		}
	}, 15);
}

function stopVolumeSmoother() {
	if (volumeSmootherTimer !== null) {
		clearInterval(volumeSmootherTimer);
		volumeSmootherTimer = null;
	}
}

function clearVolumeResetTimer(ch) {
	if (volumeResetTimers[ch] !== null) {
		clearTimeout(volumeResetTimers[ch]);
		volumeResetTimers[ch] = null;
	}
}

function clearAllVolumeResetTimers() {
	for (var ch = 0; ch < 16; ch++) {
		clearVolumeResetTimer(ch);
	}
}

function scheduleVolumeResetAfterNoteOn(ch) {
	clearVolumeResetTimer(ch);
	ccSeenSinceNoteOn[ch] = false;

	volumeResetTimers[ch] = setTimeout(function () {
		volumeResetTimers[ch] = null;

		if (!ccSeenSinceNoteOn[ch]) {
			setChannelVolume(ch, 127);
		}
	}, RESET_DELAY_MS);
}

function noteCCSeen(ch) {
	ccSeenSinceNoteOn[ch] = true;
	clearVolumeResetTimer(ch);
}

function initializeChannels() {
	if (!synth) return;

	for (var ch = 0; ch < 16; ch++) {
		synth.programChange(ch, synthPresetNumber);

		volumeValue[ch] = 127;
		volumeTarget[ch] = 127;
		volumeCurrent[ch] = 127;
		ccSeenSinceNoteOn[ch] = false;
		clearVolumeResetTimer(ch);

		activeNotesByChannel[ch] = {};
		lastPitchBendValue[ch] = 8192;
		lastChannelPressureValue[ch] = 0;
		lastCCValue[ch] = {
			7: 127,
			11: 127,
			91: synthReverbValue
		};

		clearPendingMidiForChannel(ch);

		// Keep channel volume fixed and smooth expression only
		control(ch, 7, 127);
		control(ch, 11, 127);
		control(ch, 91, synthReverbValue);

		pb(ch, 8192);
		channelPressure(ch, 0);
	}

	setPitchBendRange(pitchBendRangeSemitones, 0);
}

async function startAudio() {
	if ("audioSession" in navigator) {
		try {
			navigator.audioSession.type = "playback";
			console.log("Audio session type set to:", navigator.audioSession.type);
		} catch (e) {
			console.warn("Could not set audio session type:", e);
		}
	}

	if (synthReady && synth && audioContext) {
		if (audioContext.state !== "running") {
			await audioContext.resume();
		}
		return;
	}

	if (startAudioPromise) {
		return startAudioPromise;
	}

	startAudioPromise = (async function () {
		if (!window.WORKLET_URL) {
			throw new Error("window.WORKLET_URL is not defined");
		}
		if (!window.SOUNDFONT_URL) {
			throw new Error("window.SOUNDFONT_URL is not defined");
		}
		if (!window.WorkletSynthesizer) {
			throw new Error("window.WorkletSynthesizer is not defined");
		}

		setStatus("Starting audio...");

		audioContext = new AudioContext();
		await audioContext.resume();

		audioContext.onstatechange = function () { // Turn off the loudspeaker button if the audiocontext has been suspended.
			if (!(audioContext && audioContext.state === "running")) {
				if (volume) {
					toggleOn();
				}
			}
		};

		setStatus("Loading synth processor...");
		await audioContext.audioWorklet.addModule(window.WORKLET_URL);

		setStatus("Loading SoundFont...");
		const response = await fetch(window.SOUNDFONT_URL, { cache: "no-store" });
		if (!response.ok) {
			throw new Error("SoundFont load failed: " + response.status + " " + response.statusText);
		}
		const sf = await response.arrayBuffer();

		synth = new window.WorkletSynthesizer(audioContext);
		await synth.soundBankManager.addSoundBank(sf, "main");
		await synth.isReady;

		connectSynthOutput();
		initializeChannels();
		startVolumeSmoother();

		synthReady = true;
		setStatus("Ready");
		setGlobalTranspose(synthTransposeValue);
	})();

	try {
		await startAudioPromise;
	} finally {
		startAudioPromise = null;
	}
}

async function stopAudio() {
	try {
		clearAllPendingMidi();
		clearAllVolumeResetTimers();
		stopVolumeSmoother();

		if (synth) {
			for (var ch = 0; ch < 16; ch++) {
				if (typeof synth.allNotesOff === "function") {
					synth.allNotesOff(ch);
				}
				activeNotesByChannel[ch] = {};
			}
		}

		disconnectSynthOutput();

		if (audioContext && audioContext.state !== "closed") {
			await audioContext.close();
		}
	} catch (e) {
		console.warn("Error stopping audio:", e);
	}

	audioContext = null;
	synth = null;
	synthReady = false;
	startAudioPromise = null;

	setStatus("Audio Off");
}

function testNote() {
	if (!synthReady || !synth) return;

	synthNoteOn(0, 74, 120);
	LED2on();

	setTimeout(function () {
		if (synth) {
			synthNoteOff(0, 74);
			LED2off();
		}
	}, 1000);
}

function handleVolumeControl(ch, value) {
	noteCCSeen(ch);
	setChannelVolume(ch, value);
}

async function toggleOn() {
	volume = +!volume;
	if (volume == 0) {
		document.getElementById("volumeOff").style.display = "block";
		document.getElementById("volumeOn").style.display = "none";
		try {
			await stopAudio();
		} catch (e) {
			console.warn(e);
		}
	} else {
		document.getElementById("volumeOff").style.display = "none";
		document.getElementById("volumeOn").style.display = "block";
		try {
			await startAudio();
		} catch (e) {
			console.warn(e);
		}
	}
}

document.getElementById("volumeOn").onclick = async function () {
	try {
		await startAudio();
	} catch (e) {
		console.error("startAudio failed:", e);
	}
};

document.getElementById("testButton").onclick = async function () {
	await startAudio();
	testNote();
};

presetSelect.onchange = function () {
	if (synthReady && synth) {
		stopAllSynthNotes();
		synthPresetNumber = presetSelect.value;
		localStorage.setItem("synthPreset", synthPresetNumber);
		initializeChannels();
	}
};

reverbSlider.oninput = function () {
	reverbValue.textContent = reverbSlider.value;
	if (synthReady && synth) {
		synthReverbValue = reverbSlider.value;
		localStorage.setItem("synthReverb", synthReverbValue);
		initializeChannels();
	}
};

reverbValue.textContent = reverbSlider.value;

// End synth