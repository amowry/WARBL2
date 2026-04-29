// src/synthesizer/basic/key_modifier_manager.ts
import { KeyModifier } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/utils/fill_with_defaults.ts
function fillWithDefaults(obj, defObj) {
  return {
    ...defObj,
    ...obj
  };
}

// src/synthesizer/basic/key_modifier_manager.ts
var WorkletKeyModifierManagerWrapper = class {
  // The velocity override mappings for MIDI keys
  keyModifiers = [];
  synth;
  constructor(synth) {
    this.synth = synth;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Modifies a single key.
   * @param channel The channel affected. Usually 0-15.
   * @param midiNote The MIDI note to change. 0-127.
   * @param options The key's modifiers.
   */
  addModifier(channel, midiNote, options) {
    const mod = new KeyModifier();
    mod.gain = options?.gain ?? 1;
    mod.velocity = options?.velocity ?? -1;
    mod.patch = fillWithDefaults(
      options.patch ?? {},
      {
        isGMGSDrum: false,
        bankLSB: -1,
        bankMSB: -1,
        program: -1
      }
    );
    this.keyModifiers[channel] ??= [];
    this.keyModifiers[channel][midiNote] = mod;
    this.sendToWorklet("addMapping", {
      channel,
      midiNote,
      mapping: mod
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Gets a key modifier.
   * @param channel The channel affected. Usually 0-15.
   * @param midiNote The MIDI note to change. 0-127.
   * @returns The key modifier if it exists.
   */
  getModifier(channel, midiNote) {
    return this.keyModifiers?.[channel]?.[midiNote];
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Deletes a key modifier.
   * @param channel The channel affected. Usually 0-15.
   * @param midiNote The MIDI note to change. 0-127.
   */
  deleteModifier(channel, midiNote) {
    this.sendToWorklet("deleteMapping", {
      channel,
      midiNote
    });
    if (this.keyModifiers[channel]?.[midiNote] === void 0) {
      return;
    }
    this.keyModifiers[channel][midiNote] = void 0;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Clears ALL Modifiers
   */
  clearModifiers() {
    this.sendToWorklet("clearMappings", null);
    this.keyModifiers = [];
  }
  sendToWorklet(type, data) {
    const msg = {
      type,
      data
    };
    this.synth.post({
      type: "keyModifierManager",
      channelNumber: -1,
      data: msg
    });
  }
};

// src/synthesizer/basic/sound_bank_manager.ts
import {
  SpessaSynthCoreUtils
} from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var SoundBankManager = class {
  /**
   * All the sound banks, ordered from the most important to the least.
   */
  soundBankList;
  synth;
  /**
   * Creates a new instance of the sound bank manager.
   */
  constructor(synth) {
    this.soundBankList = [];
    this.synth = synth;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * The current sound bank priority order.
   * @returns The IDs of the sound banks in the current order.
   */
  get priorityOrder() {
    return this.soundBankList.map((s) => s.id);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Rearranges the sound banks in a given order.
   * @param newList The order of sound banks, a list of identifiers, first overwrites second.
   */
  set priorityOrder(newList) {
    this.sendToWorklet("rearrangeSoundBanks", newList);
    this.soundBankList.sort(
      (a, b) => newList.indexOf(a.id) - newList.indexOf(b.id)
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Adds a new sound bank buffer with a given ID.
   * @param soundBankBuffer The sound bank's buffer
   * @param id The sound bank's unique identifier.
   * @param bankOffset The sound bank's bank offset. Default is 0.
   */
  async addSoundBank(soundBankBuffer, id, bankOffset = 0) {
    this.sendToWorklet(
      "addSoundBank",
      {
        soundBankBuffer,
        bankOffset,
        id
      },
      [soundBankBuffer]
    );
    await this.awaitResponse();
    const found = this.soundBankList.find((s) => s.id === id);
    if (found === void 0) {
      this.soundBankList.push({
        id,
        bankOffset
      });
    } else {
      found.bankOffset = bankOffset;
    }
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Deletes a sound bank with the given ID.
   * @param id The sound bank to delete.
   */
  async deleteSoundBank(id) {
    if (this.soundBankList.length < 2) {
      SpessaSynthCoreUtils.SpessaSynthWarn(
        "1 sound bank left. Aborting!"
      );
      return;
    }
    if (!this.soundBankList.some((s) => s.id === id)) {
      SpessaSynthCoreUtils.SpessaSynthWarn(
        `No sound banks with id of "${id}" found. Aborting!`
      );
      return;
    }
    this.sendToWorklet("deleteSoundBank", id);
    this.soundBankList = this.soundBankList.filter((s) => s.id !== id);
    await this.awaitResponse();
  }
  async awaitResponse() {
    return new Promise(
      (r) => this.synth.awaitWorkerResponse("soundBankManager", r)
    );
  }
  sendToWorklet(type, data, transferable = []) {
    const msg = {
      type: "soundBankManager",
      channelNumber: -1,
      data: {
        type,
        data
      }
    };
    this.synth.post(msg, transferable);
  }
};

// src/synthesizer/basic/synth_event_handler.ts
var SynthEventHandler = class {
  /**
   * The time delay before an event is called.
   * Set to 0 to disable it.
   */
  timeDelay = 0;
  /**
   * The main list of events.
   * @private
   */
  events = {
    noteOff: /* @__PURE__ */ new Map(),
    // Called on a note off message
    noteOn: /* @__PURE__ */ new Map(),
    // Called on a note on message
    pitchWheel: /* @__PURE__ */ new Map(),
    // Called on a pitch-wheel change
    controllerChange: /* @__PURE__ */ new Map(),
    // Called on a controller change
    programChange: /* @__PURE__ */ new Map(),
    // Called on a program change
    channelPressure: /* @__PURE__ */ new Map(),
    // Called on a channel pressure message
    polyPressure: /* @__PURE__ */ new Map(),
    // Called on a poly pressure message
    drumChange: /* @__PURE__ */ new Map(),
    // Called when a channel type changes
    stopAll: /* @__PURE__ */ new Map(),
    // Called when the synth receives stop all command
    newChannel: /* @__PURE__ */ new Map(),
    // Called when a new channel is created
    muteChannel: /* @__PURE__ */ new Map(),
    // Called when a channel is muted/unmuted
    presetListChange: /* @__PURE__ */ new Map(),
    // Called when the preset list changes (soundfont gets reloaded)
    allControllerReset: /* @__PURE__ */ new Map(),
    // Called when all controllers are reset
    soundBankError: /* @__PURE__ */ new Map(),
    // Called when a sound bank parsing error occurs
    synthDisplay: /* @__PURE__ */ new Map(),
    // Called when there's a SysEx message to display some text
    masterParameterChange: /* @__PURE__ */ new Map(),
    // Called when a master parameter changes
    channelPropertyChange: /* @__PURE__ */ new Map(),
    // Called when a channel property changes
    effectChange: /* @__PURE__ */ new Map()
    // Called when an effect processor parameter is changed
  };
  /**
   * Adds a new event listener.
   * @param event The event to listen to.
   * @param id The unique identifier for the event. It can be used to overwrite existing callback with the same ID.
   * @param callback The callback for the event.
   */
  addEvent(event, id, callback) {
    this.events[event].set(id, callback);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Removes an event listener
   * @param name The event to remove a listener from.
   * @param id The unique identifier for the event to remove.
   */
  removeEvent(name, id) {
    this.events[name].delete(id);
  }
  /**
   * Calls the given event.
   * INTERNAL USE ONLY!
   * @internal
   */
  callEventInternal(name, eventData) {
    const eventList = this.events[name];
    const callback = () => {
      for (const callback2 of eventList.values()) {
        try {
          callback2(eventData);
        } catch (error) {
          console.error(
            `Error while executing an event callback for ${name}:`,
            error
          );
        }
      }
    };
    if (this.timeDelay > 0) {
      setTimeout(callback.bind(this), this.timeDelay * 1e3);
    } else {
      callback();
    }
  }
};

// src/synthesizer/basic/basic_synthesizer.ts
import {
  ALL_CHANNELS_OR_DIFFERENT_ACTION,
  DEFAULT_MASTER_PARAMETERS,
  DEFAULT_PERCUSSION,
  midiControllers,
  midiMessageTypes,
  SpessaSynthCoreUtils as util
} from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/utils/other.ts
import { SpessaSynthCoreUtils as SpessaSynthCoreUtils2 } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var consoleColors = SpessaSynthCoreUtils2.consoleColors;

// src/synthesizer/basic/snapshot.ts
import { SynthesizerSnapshot } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/synthesizer/basic/basic_synthesizer.ts
var DEFAULT_SYNTH_METHOD_OPTIONS = {
  time: 0
};
var BasicSynthesizer = class {
  /**
   * Allows managing the sound bank list.
   */
  soundBankManager = new SoundBankManager(this);
  /**
   * Allows managing key modifications.
   */
  keyModifierManager = new WorkletKeyModifierManagerWrapper(
    this
  );
  /**
   * Allows setting up custom event listeners for the synthesizer.
   */
  eventHandler = new SynthEventHandler();
  /**
   * Synthesizer's parent AudioContext instance.
   */
  context;
  /**
   * Synth's current channel properties.
   */
  channelProperties = [];
  /**
   * The current preset list.
   */
  presetList = [];
  /**
   * INTERNAL USE ONLY!
   * @internal
   * All sequencer callbacks
   */
  sequencers = new Array();
  /**
   * Resolves when the synthesizer is ready.
   */
  isReady;
  // noinspection JSUnusedGlobalSymbols
  /**
   * Legacy parameter.
   * @deprecated
   */
  reverbProcessor = void 0;
  // noinspection JSUnusedGlobalSymbols
  /**
   * Legacy parameter.
   * @deprecated
   */
  chorusProcessor = void 0;
  /**
   * INTERNAL USE ONLY!
   * @internal
   */
  post;
  worklet;
  /**
   * The new channels will have their audio sent to the modulated output by this constant.
   * what does that mean?
   * e.g., if outputsAmount is 16, then channel's 16 audio data will be sent to channel 0
   */
  _outputsAmount = 16;
  /**
   * The current amount of MIDI channels the synthesizer has.
   */
  channelsAmount = this._outputsAmount;
  masterParameters = {
    ...DEFAULT_MASTER_PARAMETERS
  };
  // Resolve map, waiting for the worklet to confirm the operation
  resolveMap = /* @__PURE__ */ new Map();
  renderingProgressTracker = /* @__PURE__ */ new Map();
  /**
   * Creates a new instance of a synthesizer.
   * @param worklet The AudioWorkletNode to use.
   * @param postFunction The internal post function.
   * @param config Optional configuration for the synthesizer.
   */
  constructor(worklet, postFunction, config) {
    util.SpessaSynthInfo(
      "%cInitializing SpessaSynth synthesizer...",
      consoleColors.info
    );
    this.context = worklet.context;
    this.worklet = worklet;
    this.post = postFunction;
    void config;
    this.isReady = new Promise(
      (resolve) => this.awaitWorkerResponse("sf3Decoder", resolve)
    );
    this.worklet.port.onmessage = (e) => this.handleMessage(e.data);
    for (let i = 0; i < this.channelsAmount; i++) {
      this.addNewChannelInternal(false);
    }
    this.channelProperties[DEFAULT_PERCUSSION].isDrum = true;
    this.eventHandler.addEvent(
      "newChannel",
      `synth-new-channel-${Math.random()}`,
      () => {
        this.channelsAmount++;
      }
    );
    this.eventHandler.addEvent(
      "presetListChange",
      `synth-preset-list-change-${Math.random()}`,
      (e) => {
        this.presetList = [...e];
      }
    );
    this.eventHandler.addEvent(
      "masterParameterChange",
      `synth-master-parameter-change-${Math.random()}`,
      (e) => {
        this.masterParameters[e.parameter] = e.value;
      }
    );
    this.eventHandler.addEvent(
      "channelPropertyChange",
      `synth-channel-property-change-${Math.random()}`,
      (e) => {
        this.channelProperties[e.channel] = e.property;
        this._voicesAmount = this.channelProperties.reduce(
          (sum, voices) => sum + voices.voicesAmount,
          0
        );
      }
    );
  }
  /**
   * Current voice amount
   */
  _voicesAmount = 0;
  // noinspection JSUnusedGlobalSymbols
  /**
   * The current number of voices playing.
   */
  get voicesAmount() {
    return this._voicesAmount;
  }
  /**
   * The audioContext's current time.
   */
  get currentTime() {
    return this.context.currentTime;
  }
  /**
   * Connects from a given node.
   * @param destinationNode The node to connect to.
   */
  connect(destinationNode) {
    for (let i = 0; i < 17; i++) {
      this.worklet.connect(destinationNode, i);
    }
    return destinationNode;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Disconnects from a given node.
   * @param destinationNode The node to disconnect from.
   */
  disconnect(destinationNode) {
    if (!destinationNode) {
      this.worklet.disconnect();
      return void 0;
    }
    for (let i = 0; i < 17; i++) {
      this.worklet.disconnect(destinationNode, i);
    }
    return destinationNode;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Sets the SpessaSynth's log level in the processor.
   * @param enableInfo Enable info (verbose)
   * @param enableWarning Enable warnings (unrecognized messages)
   * @param enableGroup Enable groups (to group a lot of logs)
   */
  setLogLevel(enableInfo, enableWarning, enableGroup) {
    this.post({
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION,
      type: "setLogLevel",
      data: {
        enableInfo,
        enableWarning,
        enableGroup
      }
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Gets a master parameter from the synthesizer.
   * @param type The parameter to get.
   * @returns The parameter value.
   */
  getMasterParameter(type) {
    return this.masterParameters[type];
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Sets a master parameter to a given value.
   * @param type The parameter to set.
   * @param value The value to set.
   */
  setMasterParameter(type, value) {
    this.masterParameters[type] = value;
    this.post({
      type: "setMasterParameter",
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION,
      data: {
        type,
        data: value
      }
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Gets a complete snapshot of the synthesizer, effects.
   */
  async getSnapshot() {
    return new Promise((resolve) => {
      this.awaitWorkerResponse("synthesizerSnapshot", (s) => {
        const snapshot = SynthesizerSnapshot.copyFrom(s);
        resolve(snapshot);
      });
      this.post({
        type: "requestSynthesizerSnapshot",
        data: null,
        channelNumber: -1
      });
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Adds a new channel to the synthesizer.
   */
  addNewChannel() {
    this.addNewChannelInternal(true);
  }
  /**
   * DEPRECATED, please don't use it!
   * @deprecated
   */
  setVibrato(channel, value) {
    void channel;
    void value;
  }
  /**
   * Connects the individual audio outputs to the given audio nodes. In the app, it's used by the renderer.
   * @param audioNodes Exactly 16 outputs.
   */
  connectIndividualOutputs(audioNodes) {
    if (audioNodes.length !== this._outputsAmount) {
      throw new Error(`input nodes amount differs from the system's outputs amount!
            Expected ${this._outputsAmount} got ${audioNodes.length}`);
    }
    for (let outputNumber = 0; outputNumber < this._outputsAmount; outputNumber++) {
      this.worklet.connect(audioNodes[outputNumber], outputNumber + 1);
    }
  }
  /**
   * Disconnects the individual audio outputs to the given audio nodes. In the app, it's used by the renderer.
   * @param audioNodes Exactly 16 outputs.
   */
  disconnectIndividualOutputs(audioNodes) {
    if (audioNodes.length !== this._outputsAmount) {
      throw new Error(`input nodes amount differs from the system's outputs amount!
            Expected ${this._outputsAmount} got ${audioNodes.length}`);
    }
    for (let outputNumber = 0; outputNumber < this._outputsAmount; outputNumber++) {
      this.worklet.disconnect(audioNodes[outputNumber], outputNumber + 2);
    }
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Disables the GS NRPN parameters like vibrato or drum key tuning.
   * @deprecated Deprecated! Please use master parameters
   */
  disableGSNPRNParams() {
    this.setMasterParameter("nprnParamLock", true);
  }
  /**
   * Sends a raw MIDI message to the synthesizer.
   * @param message the midi message, each number is a byte.
   * @param channelOffset the channel offset of the message.
   * @param eventOptions additional options for this command.
   */
  sendMessage(message, channelOffset = 0, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    this._sendInternal(message, channelOffset, false, eventOptions);
  }
  /**
   * Starts playing a note
   * @param channel Usually 0-15: the channel to play the note.
   * @param midiNote 0-127 the key number of the note.
   * @param velocity 0-127 the velocity of the note (generally controls loudness).
   * @param eventOptions Additional options for this command.
   */
  noteOn(channel, midiNote, velocity, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    const ch = channel % 16;
    const offset = channel - ch;
    midiNote %= 128;
    velocity %= 128;
    this.sendMessage(
      [midiMessageTypes.noteOn | ch, midiNote, velocity],
      offset,
      eventOptions
    );
  }
  /**
   * Stops playing a note.
   * @param channel Usually 0-15: the channel of the note.
   * @param midiNote {number} 0-127 the key number of the note.
   * @param force Instantly kills the note if true.
   * @param eventOptions Additional options for this command.
   */
  noteOff(channel, midiNote, force = false, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    midiNote %= 128;
    const ch = channel % 16;
    const offset = channel - ch;
    this._sendInternal(
      [midiMessageTypes.noteOff | ch, midiNote],
      offset,
      force,
      eventOptions
    );
  }
  /**
   * Stops all notes.
   * @param force If the notes should immediately be stopped, defaults to false.
   */
  stopAll(force = false) {
    this.post({
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION,
      type: "stopAll",
      data: force ? 1 : 0
    });
  }
  /**
   * Changes the given controller
   * @param channel Usually 0-15: the channel to change the controller.
   * @param controllerNumber 0-127 the MIDI CC number.
   * @param controllerValue 0-127 the controller value.
   * @param force Forces the controller-change message, even if it's locked or gm system is set and the cc is bank select.
   * @param eventOptions Additional options for this command.
   */
  controllerChange(channel, controllerNumber, controllerValue, force = false, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    if (controllerNumber > 127 || controllerNumber < 0) {
      throw new Error(`Invalid controller number: ${controllerNumber}`);
    }
    controllerValue = Math.floor(controllerValue) % 128;
    controllerNumber = Math.floor(controllerNumber) % 128;
    const ch = channel % 16;
    const offset = channel - ch;
    this._sendInternal(
      [
        midiMessageTypes.controllerChange | ch,
        controllerNumber,
        controllerValue
      ],
      offset,
      force,
      eventOptions
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Resets all controllers (for every channel)
   */
  resetControllers() {
    this.post({
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION,
      type: "ccReset",
      data: null
    });
  }
  /**
   * Causes the given midi channel to ignore controller messages for the given controller number.
   * @param channel Usually 0-15: the channel to lock.
   * @param controllerNumber 0-127 MIDI CC number.
   * @param isLocked True if locked, false if unlocked.
   * @remarks
   *  Controller number -1 locks the preset.
   */
  lockController(channel, controllerNumber, isLocked) {
    this.post({
      channelNumber: channel,
      type: "lockController",
      data: {
        controllerNumber,
        isLocked
      }
    });
  }
  /**
   * Applies pressure to a given channel.
   * @param channel Usually 0-15: the channel to change the controller.
   * @param pressure 0-127: the pressure to apply.
   * @param eventOptions Additional options for this command.
   */
  channelPressure(channel, pressure, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    const ch = channel % 16;
    const offset = channel - ch;
    pressure %= 128;
    this.sendMessage(
      [midiMessageTypes.channelPressure | ch, pressure],
      offset,
      eventOptions
    );
  }
  /**
   * Applies pressure to a given note.
   * @param channel Usually 0-15: the channel to change the controller.
   * @param midiNote 0-127: the MIDI note.
   * @param pressure 0-127: the pressure to apply.
   * @param eventOptions Additional options for this command.
   */
  polyPressure(channel, midiNote, pressure, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    const ch = channel % 16;
    const offset = channel - ch;
    midiNote %= 128;
    pressure %= 128;
    this.sendMessage(
      [midiMessageTypes.polyPressure | ch, midiNote, pressure],
      offset,
      eventOptions
    );
  }
  /**
   * Sets the pitch of the given channel.
   * @param channel Usually 0-15: the channel to change pitch.
   * @param value The bend of the MIDI pitch wheel message. 0 - 16384
   * @param eventOptions Additional options for this command.
   */
  pitchWheel(channel, value, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    const ch = channel % 16;
    const offset = channel - ch;
    this.sendMessage(
      [midiMessageTypes.pitchWheel | ch, value & 127, value >> 7],
      offset,
      eventOptions
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Sets the channel's pitch wheel range, in semitones.
   * @param channel Usually 0-15: the channel to change.
   * @param range The bend range in semitones.
   */
  pitchWheelRange(channel, range) {
    this.controllerChange(
      channel,
      midiControllers.registeredParameterMSB,
      0
    );
    this.controllerChange(
      channel,
      midiControllers.registeredParameterLSB,
      0
    );
    this.controllerChange(channel, midiControllers.dataEntryMSB, range);
    this.controllerChange(
      channel,
      midiControllers.registeredParameterMSB,
      127
    );
    this.controllerChange(
      channel,
      midiControllers.registeredParameterLSB,
      127
    );
    this.controllerChange(channel, midiControllers.dataEntryMSB, 0);
  }
  /**
   * Changes the program for a given channel
   * @param channel Usually 0-15: the channel to change.
   * @param programNumber 0-127 the MIDI patch number.
   * defaults to false
   */
  programChange(channel, programNumber) {
    const ch = channel % 16;
    const offset = channel - ch;
    programNumber %= 128;
    this.sendMessage(
      [midiMessageTypes.programChange | ch, programNumber],
      offset
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Transposes the channel by given number of semitones.
   * @param channel The channel number.
   * @param semitones The transposition of the channel, it can be a float.
   * @param force Defaults to false, if true transposes the channel even if it's a drum channel.
   */
  transposeChannel(channel, semitones, force = false) {
    this.post({
      channelNumber: channel,
      type: "transposeChannel",
      data: {
        semitones,
        force
      }
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Mutes or unmutes the given channel.
   * @param channel Usually 0-15: the channel to mute.
   * @param isMuted Indicates if the channel is muted.
   */
  muteChannel(channel, isMuted) {
    this.post({
      channelNumber: channel,
      type: "muteChannel",
      data: isMuted
    });
  }
  /**
   * Sends a MIDI Sysex message to the synthesizer.
   * @param messageData The message's data, excluding the F0 byte, but including the F7 at the end.
   * @param channelOffset Channel offset for the system exclusive message, defaults to zero.
   * @param eventOptions Additional options for this command.
   */
  systemExclusive(messageData, channelOffset = 0, eventOptions = DEFAULT_SYNTH_METHOD_OPTIONS) {
    this._sendInternal(
      [midiMessageTypes.systemExclusive, ...Array.from(messageData)],
      channelOffset,
      false,
      eventOptions
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Tune MIDI keys of a given program using the MIDI Tuning Standard.
   * @param program  0 - 127 the MIDI program number to use.
   * @param tunings The keys and their tunings.
   * TargetPitch of -1 sets the tuning for this key to be tuned regularly.
   */
  tuneKeys(program, tunings) {
    if (tunings.length > 127) {
      throw new Error("Too many tunings. Maximum allowed is 127.");
    }
    const systemExclusive = [
      127,
      // Real-time
      16,
      // Device id
      8,
      // MIDI Tuning
      2,
      // Note change
      program,
      // Tuning program number
      tunings.length
      // Number of changes
    ];
    for (const tuning of tunings) {
      systemExclusive.push(tuning.sourceKey);
      if (tuning.targetPitch === -1) {
        systemExclusive.push(127, 127, 127);
      } else {
        const midiNote = Math.floor(tuning.targetPitch);
        const fraction = Math.floor(
          (tuning.targetPitch - midiNote) / 61e-6
        );
        systemExclusive.push(
          midiNote,
          // Frequency data byte 1
          fraction >> 7 & 127,
          // Frequency data byte 2
          fraction & 127
          // Frequency data byte 3
        );
      }
    }
    systemExclusive.push(247);
    this.systemExclusive(systemExclusive);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Toggles drums on a given channel.
   * @param channel The channel number.
   * @param isDrum If the channel should be drums.
   */
  setDrums(channel, isDrum) {
    this.post({
      channelNumber: channel,
      type: "setDrums",
      data: isDrum
    });
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Yes please!
   */
  reverbateEverythingBecauseWhyNot() {
    for (let i = 0; i < this.channelsAmount; i++) {
      this.controllerChange(i, midiControllers.reverbDepth, 127);
      this.lockController(i, midiControllers.reverbDepth, true);
    }
    return "That's the spirit!";
  }
  /**
   * INTERNAL USE ONLY!
   * @param type INTERNAL USE ONLY!
   * @param resolve INTERNAL USE ONLY!
   * @internal
   */
  awaitWorkerResponse(type, resolve) {
    this.resolveMap.set(type, resolve);
  }
  /**
   * INTERNAL USE ONLY!
   * @param callback the sequencer callback
   * @internal
   */
  assignNewSequencer(callback) {
    this.post({
      channelNumber: -1,
      type: "requestNewSequencer",
      data: null
    });
    this.sequencers.push(callback);
    return this.sequencers.length - 1;
  }
  assignProgressTracker(type, progressFunction) {
    if (this.renderingProgressTracker.get(type)) {
      throw new Error("Something is already being rendered!");
    }
    this.renderingProgressTracker.set(type, progressFunction);
  }
  revokeProgressTracker(type) {
    this.renderingProgressTracker.delete(type);
  }
  _sendInternal(message, channelOffset, force = false, eventOptions) {
    const options = fillWithDefaults(
      eventOptions,
      DEFAULT_SYNTH_METHOD_OPTIONS
    );
    this.post({
      type: "midiMessage",
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION,
      data: {
        messageData: new Uint8Array(message),
        channelOffset,
        force,
        options
      }
      //[new Uint8Array(message), offset, force, opts]
    });
  }
  /**
   * Handles the messages received from the worklet.
   */
  handleMessage(m) {
    switch (m.type) {
      case "eventCall": {
        this.eventHandler.callEventInternal(m.data.type, m.data.data);
        break;
      }
      case "sequencerReturn": {
        this.sequencers[m.data.id](m.data);
        break;
      }
      case "isFullyInitialized": {
        this.workletResponds(m.data.type, m.data.data);
        break;
      }
      case "soundBankError": {
        util.SpessaSynthWarn(m.data);
        this.eventHandler.callEventInternal("soundBankError", m.data);
        break;
      }
      case "renderingProgress": {
        this.renderingProgressTracker.get(m.data.type)?.(
          // @ts-expect-error I can't use generics with map
          m.data.data
        );
      }
    }
  }
  addNewChannelInternal(post) {
    this.channelProperties.push({
      voicesAmount: 0,
      pitchWheel: 0,
      pitchWheelRange: 0,
      isMuted: false,
      isDrum: this.channelsAmount % 16 === DEFAULT_PERCUSSION,
      isEFX: false,
      transposition: 0
    });
    if (!post) {
      return;
    }
    this.post({
      channelNumber: 0,
      type: "addNewChannel",
      data: null
    });
  }
  workletResponds(type, data) {
    this.resolveMap.get(type)?.(data);
    this.resolveMap.delete(type);
  }
};

// src/synthesizer/basic/synth_config.ts
var DEFAULT_SYNTH_CONFIG = {
  enableEventSystem: true,
  oneOutput: false,
  audioNodeCreators: void 0
};

// src/synthesizer/worklet/worklet_processor_name.ts
var WORKLET_PROCESSOR_NAME = "spessasynth-worklet-processor";

// src/synthesizer/worklet/worklet_synthesizer.ts
var WorkletSynthesizer = class extends BasicSynthesizer {
  /**
   * Creates a new instance of an AudioWorklet-based synthesizer.
   * @param context The audio context.
   * @param config Optional configuration for the synthesizer.
   */
  constructor(context, config = DEFAULT_SYNTH_CONFIG) {
    const synthConfig = fillWithDefaults(config, DEFAULT_SYNTH_CONFIG);
    let outputChannelCount = new Array(17).fill(2);
    let numberOfOutputs = 17;
    if (synthConfig.oneOutput) {
      outputChannelCount = [34];
      numberOfOutputs = 1;
    }
    let worklet;
    try {
      const workletConstructor = synthConfig?.audioNodeCreators?.worklet ?? ((context2, name, options) => {
        return new AudioWorkletNode(context2, name, options);
      });
      worklet = workletConstructor(context, WORKLET_PROCESSOR_NAME, {
        outputChannelCount,
        numberOfOutputs,
        processorOptions: {
          oneOutput: synthConfig.oneOutput,
          enableEventSystem: synthConfig.enableEventSystem
        }
      });
    } catch (error) {
      console.error(error);
      throw new Error(
        "Could not create the AudioWorkletNode. Did you forget to addModule()?"
      );
    }
    super(
      worklet,
      (data, transfer = []) => {
        worklet.port.postMessage(data, transfer);
      },
      synthConfig
    );
  }
  /**
   * Starts an offline audio render.
   * @param config The configuration to use.
   * @remarks
   * Call this method immediately after you've set up the synthesizer.
   * Do NOT call any other methods after initializing before this one.
   * Chromium seems to ignore worklet messages for OfflineAudioContext.
   */
  async startOfflineRender(config) {
    this.post(
      {
        type: "startOfflineRender",
        data: config,
        channelNumber: -1
      },
      config.soundBankList.map((b) => b.soundBankBuffer)
    );
    await new Promise(
      (r) => this.awaitWorkerResponse("startOfflineRender", r)
    );
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Destroys the synthesizer instance.
   */
  destroy() {
    this.post({
      channelNumber: 0,
      type: "destroyWorklet",
      data: null
    });
    this.worklet.disconnect();
    delete this.worklet;
  }
};

// src/synthesizer/worker/playback_worklet.ts
var PLAYBACK_WORKLET_PROCESSOR_NAME = `spessasynth-playback-worklet-processor`;
function getPlaybackWorkletURL(maxQueuedChunks) {
  const PLAYBACK_WORKLET_CODE = `
const BLOCK_SIZE = 128;

const MAX_QUEUED = ${maxQueuedChunks};

/**
 * An AudioWorkletProcessor that plays back 18 separate streams of stereo audio: reverb, and chorus and 16 dry channels.
 */
class PlaybackProcessor extends AudioWorkletProcessor
{
    
    
    /** @type {Float32Array[]} */
    data = [];
    
    updateRequested = false;
    
    alive = true;
    
    /**
     *
     * @type {MessagePort}
     */
    sentPort;
    
    constructor()
    {
        super();
        
        /**
         * @param e {MessageEvent}
         */
        this.port.onmessage = (e) =>
        {
            if (e.ports.length)
            {
                const sentPort = e.ports[0];
                this.sentPort = sentPort;
                sentPort.onmessage = (e) =>
                {
                    if(e.data === null)
                    {
                        // the worklet is dead
                        this.alive = false;
                    }
                    this.data.push(e.data);
                    this.updateRequested = false;
                    // if we need more, request immediately
                    if (this.data.length < MAX_QUEUED)
                    {
                        this.sentPort.postMessage(null);
                    }
                };
                
            }
        };
    }
    
    // noinspection JSUnusedGlobalSymbols
    /**
     * @param _inputs {[Float32Array, Float32Array][]}
     * @param outputs {[Float32Array, Float32Array][]}
     * @returns {boolean}
     */
    process(_inputs, outputs)
    {
        const data = this.data.shift();
        if (!data)
        {
            return this.alive;
        }
        let offset = 0;
        // decode the data nicely
        for (let i = 0; i < 17; i++)
        {
            outputs[i][0].set(data.subarray(offset, offset + BLOCK_SIZE));
            offset += BLOCK_SIZE;
            outputs[i][1].set(data.subarray(offset, offset + BLOCK_SIZE));
            offset += BLOCK_SIZE;
        }
        
        // if it has already been requested, we need to wait
        if (!this.updateRequested)
        {
            this.sentPort.postMessage(null);
            this.updateRequested = true;
        }
        
        // keep it online
        return this.alive;
    }
}
registerProcessor("${PLAYBACK_WORKLET_PROCESSOR_NAME}", PlaybackProcessor);
    `;
  const blob = new Blob([PLAYBACK_WORKLET_CODE], {
    type: "application/javascript"
  });
  return URL.createObjectURL(blob);
}

// src/synthesizer/worker/render_audio_worker.ts
import { SpessaSynthProcessor, SpessaSynthSequencer } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var DEFAULT_WORKER_RENDER_AUDIO_OPTIONS = {
  extraTime: 2,
  separateChannels: false,
  loopCount: 0,
  progressCallback: void 0,
  preserveSynthParams: true,
  enableEffects: true,
  sequencerID: 0
};
var RENDER_BLOCKS_PER_PROGRESS = 64;
var BLOCK_SIZE = 128;
function renderAudioWorker(sampleRate, options) {
  const rendererSynth = new SpessaSynthProcessor(sampleRate, {
    enableEventSystem: false
  });
  const rendererSeq = new SpessaSynthSequencer(rendererSynth);
  rendererSynth.setMasterParameter("autoAllocateVoices", true);
  for (const entry of this.synthesizer.soundBankManager.soundBankList)
    rendererSynth.soundBankManager.addSoundBank(
      entry.soundBank,
      entry.id,
      entry.bankOffset
    );
  rendererSynth.soundBankManager.priorityOrder = this.synthesizer.soundBankManager.priorityOrder;
  this.stopAudioLoop();
  const seq = this.sequencers[options.sequencerID];
  const parsedMid = seq.midiData;
  if (!parsedMid) {
    throw new Error("No MIDI is loaded!");
  }
  const playbackRate = seq.playbackRate;
  const loopStartAbsolute = parsedMid.midiTicksToSeconds(parsedMid.loop.start) / playbackRate;
  const loopEndAbsolute = parsedMid.midiTicksToSeconds(parsedMid.loop.end) / playbackRate;
  const loopDuration = loopEndAbsolute - loopStartAbsolute;
  const duration = parsedMid.duration / playbackRate + options.extraTime + loopDuration * options.loopCount;
  const sampleDuration = sampleRate * duration;
  rendererSeq.loopCount = options.loopCount;
  if (options.preserveSynthParams) {
    rendererSeq.playbackRate = seq.playbackRate;
    const snapshot = this.synthesizer.getSnapshot();
    rendererSynth.applySynthesizerSnapshot(snapshot);
  }
  rendererSeq.loadNewSongList([parsedMid]);
  rendererSeq.play();
  const wetL = new Float32Array(sampleDuration);
  const wetR = new Float32Array(sampleDuration);
  const effects = [wetL, wetR];
  const returnedChunks = {
    effects,
    dry: []
  };
  const sampleDurationNoLastQuantum = sampleDuration - BLOCK_SIZE;
  if (options.separateChannels) {
    const dry = [];
    for (let i = 0; i < 16; i++) {
      const d = [
        new Float32Array(sampleDuration),
        new Float32Array(sampleDuration)
      ];
      dry.push(d);
      returnedChunks.dry.push(d);
    }
    let index = 0;
    while (true) {
      for (let i = 0; i < RENDER_BLOCKS_PER_PROGRESS; i++) {
        if (index >= sampleDurationNoLastQuantum) {
          rendererSeq.processTick();
          rendererSynth.processSplit(
            dry,
            wetL,
            wetR,
            index,
            sampleDuration - index
          );
          this.startAudioLoop();
          return returnedChunks;
        }
        rendererSeq.processTick();
        rendererSynth.processSplit(dry, wetL, wetR, index, BLOCK_SIZE);
        index += BLOCK_SIZE;
      }
      this.postProgress("renderAudio", index / sampleDuration);
    }
  } else {
    const dryL = new Float32Array(sampleDuration);
    const dryR = new Float32Array(sampleDuration);
    const dry = [dryL, dryR];
    returnedChunks.dry.push(dry);
    let index = 0;
    while (true) {
      for (let i = 0; i < RENDER_BLOCKS_PER_PROGRESS; i++) {
        if (index >= sampleDurationNoLastQuantum) {
          rendererSeq.processTick();
          rendererSynth.process(
            dryL,
            dryR,
            index,
            sampleDuration - index
          );
          this.startAudioLoop();
          return returnedChunks;
        }
        rendererSeq.processTick();
        rendererSynth.process(dryL, dryR, index, BLOCK_SIZE);
        index += BLOCK_SIZE;
      }
      this.postProgress("renderAudio", index / sampleDuration);
    }
  }
}

// src/synthesizer/worker/worker_synthesizer.ts
var DEFAULT_BANK_WRITE_OPTIONS = {
  trim: true,
  bankID: "",
  writeEmbeddedSoundBank: true,
  sequencerID: 0
};
var DEFAULT_SF2_WRITE_OPTIONS = {
  ...DEFAULT_BANK_WRITE_OPTIONS,
  writeDefaultModulators: true,
  writeExtendedLimits: true,
  compress: false,
  compressionQuality: 1,
  decompress: false
};
var DEFAULT_RMIDI_WRITE_OPTIONS = {
  ...DEFAULT_BANK_WRITE_OPTIONS,
  bankOffset: 0,
  correctBankOffset: true,
  metadata: {},
  format: "sf2",
  ...DEFAULT_SF2_WRITE_OPTIONS
};
var DEFAULT_DLS_WRITE_OPTIONS = {
  ...DEFAULT_BANK_WRITE_OPTIONS
};
var WorkerSynthesizer = class extends BasicSynthesizer {
  /**
   * Time offset for syncing with the synth
   * @private
   */
  timeOffset = 0;
  /**
   * Creates a new instance of a Worker-based synthesizer.
   * @param context The audio context.
   * @param workerPostMessage The postMessage for the worker containing the synthesizer core.
   * @param config Optional configuration for the synthesizer.
   */
  constructor(context, workerPostMessage, config = DEFAULT_SYNTH_CONFIG) {
    const synthConfig = fillWithDefaults(config, DEFAULT_SYNTH_CONFIG);
    if (synthConfig.oneOutput) {
      throw new Error(
        "One output mode is not supported in the WorkerSynthesizer."
      );
    }
    let worklet;
    try {
      const workletConstructor = synthConfig?.audioNodeCreators?.worklet ?? ((context2, name, options) => {
        return new AudioWorkletNode(context2, name, options);
      });
      worklet = workletConstructor(
        context,
        PLAYBACK_WORKLET_PROCESSOR_NAME,
        {
          outputChannelCount: new Array(18).fill(2),
          numberOfOutputs: 18,
          processorOptions: {
            oneOutput: synthConfig.oneOutput,
            enableEventSystem: synthConfig.enableEventSystem
          }
        }
      );
    } catch (error) {
      console.error(error);
      throw new Error(
        "Could not create the AudioWorkletNode. Did you forget to registerPlaybackWorklet()?"
      );
    }
    super(
      worklet,
      workerPostMessage,
      synthConfig
    );
    const messageChannel = new MessageChannel();
    const workerPort = messageChannel.port1;
    const workletPort = messageChannel.port2;
    this.worklet.port.postMessage(null, [workletPort]);
    workerPostMessage(
      {
        initialTime: this.context.currentTime,
        sampleRate: this.context.sampleRate
      },
      [workerPort]
    );
  }
  get currentTime() {
    return this.context.currentTime + this.timeOffset;
  }
  /**
   * Registers an audio worklet for the WorkerSynthesizer.
   * @param context The context to register the worklet for.
   * @param maxQueueSize The maximum amount of 128-sample chunks to store in the worklet. Higher values result in less breakups but higher latency.
   */
  static async registerPlaybackWorklet(context, maxQueueSize = 20) {
    if (!context?.audioWorklet.addModule) {
      throw new Error("Audio worklet is not supported.");
    }
    return context.audioWorklet.addModule(
      getPlaybackWorkletURL(maxQueueSize)
    );
  }
  /**
   * Handles a return message from the Worker.
   * @param e The event received from the Worker.
   */
  handleWorkerMessage(e) {
    this.timeOffset = e.currentTime - this.context.currentTime;
    this.handleMessage(e);
  }
  /**
   * Writes a DLS file directly in the worker.
   * @param options Options for writing the file.
   * @returns The file array buffer and its corresponding name.
   */
  async writeDLS(options = DEFAULT_DLS_WRITE_OPTIONS) {
    const writeOptions = fillWithDefaults(
      options,
      DEFAULT_DLS_WRITE_OPTIONS
    );
    return new Promise((resolve) => {
      this.assignProgressTracker("workerSynthWriteFile", (p) => {
        void options.progressFunction?.(p);
      });
      const postOptions = {
        ...writeOptions,
        progressFunction: null
      };
      this.awaitWorkerResponse(
        "workerSynthWriteFile",
        (data) => resolve(data)
      );
      this.post({
        type: "writeDLS",
        data: postOptions,
        channelNumber: -1
      });
    });
  }
  /**
   * Writes an SF2/SF3 file directly in the worker.
   * @param options Options for writing the file.
   * @returns The file array buffer and its corresponding name.
   */
  async writeSF2(options = DEFAULT_SF2_WRITE_OPTIONS) {
    const writeOptions = fillWithDefaults(
      options,
      DEFAULT_SF2_WRITE_OPTIONS
    );
    return new Promise((resolve) => {
      this.assignProgressTracker("workerSynthWriteFile", (p) => {
        void options.progressFunction?.(p);
      });
      const postOptions = {
        ...writeOptions,
        progressFunction: null
      };
      this.awaitWorkerResponse(
        "workerSynthWriteFile",
        (data) => resolve(data)
      );
      this.post({
        type: "writeSF2",
        data: postOptions,
        channelNumber: -1
      });
    });
  }
  /**
   * Writes an embedded MIDI (RMIDI) file directly in the worker.
   * @param options Options for writing the file.
   * @returns The file array buffer.
   */
  async writeRMIDI(options = DEFAULT_RMIDI_WRITE_OPTIONS) {
    const writeOptions = fillWithDefaults(
      options,
      DEFAULT_RMIDI_WRITE_OPTIONS
    );
    return new Promise((resolve) => {
      this.assignProgressTracker("workerSynthWriteFile", (p) => {
        void options.progressFunction?.(p);
      });
      const postOptions = {
        ...writeOptions,
        progressFunction: null
      };
      this.awaitWorkerResponse(
        "workerSynthWriteFile",
        (data) => resolve(data.binary)
      );
      this.post({
        type: "writeRMIDI",
        data: postOptions,
        channelNumber: -1
      });
    });
  }
  /**
   * Renders the current song in the connected sequencer to Float32 buffers.
   * @param sampleRate The sample rate to use, in Hertz.
   * @param renderOptions Extra options for the render.
   * @returns A single audioBuffer if separate channels were not enabled, otherwise 16.
   * @remarks
   * This stops the synthesizer.
   */
  async renderAudio(sampleRate, renderOptions = DEFAULT_WORKER_RENDER_AUDIO_OPTIONS) {
    const options = fillWithDefaults(
      renderOptions,
      DEFAULT_WORKER_RENDER_AUDIO_OPTIONS
    );
    if (options.enableEffects && options.separateChannels) {
      throw new Error("Effects cannot be applied to separate channels.");
    }
    return new Promise((resolve) => {
      this.awaitWorkerResponse("renderAudio", (data) => {
        this.revokeProgressTracker("renderAudio");
        const bufferLength = data.dry[0][0].length;
        const dryChannels = data.dry.map((dryPair) => {
          const buffer = new AudioBuffer({
            sampleRate,
            numberOfChannels: 2,
            length: bufferLength
          });
          buffer.copyToChannel(
            dryPair[0],
            0
          );
          buffer.copyToChannel(
            dryPair[1],
            1
          );
          return buffer;
        });
        if (options.enableEffects) {
          const buffer = new AudioBuffer({
            sampleRate,
            numberOfChannels: 2,
            length: bufferLength
          });
          buffer.copyToChannel(
            data.effects[0],
            0
          );
          buffer.copyToChannel(
            data.effects[1],
            1
          );
          dryChannels.push(buffer);
        }
        resolve(dryChannels);
        return;
      });
      this.assignProgressTracker("renderAudio", (p) => {
        options.progressCallback?.(p, 0);
      });
      const strippedOptions = {
        ...options,
        progressCallback: void 0
      };
      this.post({
        type: "renderAudio",
        data: {
          sampleRate,
          options: strippedOptions
        },
        channelNumber: -1
      });
    });
  }
};

// src/synthesizer/worker/worker_synthesizer_core.ts
import { SoundBankLoader as SoundBankLoader2 } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/synthesizer/basic/basic_synthesizer_core.ts
import {
  ALL_CHANNELS_OR_DIFFERENT_ACTION as ALL_CHANNELS_OR_DIFFERENT_ACTION2,
  BasicMIDI as BasicMIDI2,
  SoundBankLoader,
  SpessaSynthCoreUtils as util2,
  SpessaSynthLogging,
  SpessaSynthProcessor as SpessaSynthProcessor2,
  SpessaSynthSequencer as SpessaSynthSequencer2,
  SynthesizerSnapshot as SynthesizerSnapshot2
} from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/sequencer/midi_data.ts
import { BasicMIDI, MIDITrack } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var MIDIDataTrack = class extends MIDITrack {
  /**
   * THIS DATA WILL BE EMPTY! USE sequencer.getMIDI() TO GET THE ACTUAL DATA!
   */
  events = [];
  constructor(track) {
    super();
    super.copyFrom(track);
    this.events = [];
  }
};
var MIDIData = class _MIDIData extends BasicMIDI {
  tracks;
  /**
   * THIS DATA WILL BE EMPTY! USE sequencer.getMIDI() TO GET THE ACTUAL DATA!
   */
  embeddedSoundBank = void 0;
  /**
   * The byte length of the sound bank if it exists.
   */
  embeddedSoundBankSize;
  constructor(mid) {
    super();
    super.copyMetadataFrom(mid);
    this.tracks = mid.tracks.map((t) => new MIDIDataTrack(t));
    this.embeddedSoundBankSize = mid instanceof _MIDIData ? mid.embeddedSoundBankSize : mid?.embeddedSoundBank?.byteLength;
  }
};

// src/sequencer/enums.ts
var songChangeType = {
  shuffleOn: 1,
  // No additional data
  shuffleOff: 2,
  // No additional data
  index: 3
  // SongIndex<number>
};

// src/synthesizer/basic/basic_synthesizer_core.ts
var SEQUENCER_SYNC_INTERVAL = 1;
var BasicSynthesizerCore = class {
  synthesizer;
  sequencers = new Array();
  post;
  lastSequencerSync = 0;
  /**
   * Indicates if the processor is alive.
   * @protected
   */
  alive = false;
  constructor(sampleRate, options, postMessage) {
    this.synthesizer = new SpessaSynthProcessor2(sampleRate, options);
    this.post = postMessage;
    this.synthesizer.onEventCall = (event) => {
      this.post({
        type: "eventCall",
        data: event,
        currentTime: this.synthesizer.currentSynthTime
      });
    };
  }
  createNewSequencer() {
    const sequencer = new SpessaSynthSequencer2(this.synthesizer);
    const sequencerID = this.sequencers.length;
    this.sequencers.push(sequencer);
    sequencer.onEventCall = (e) => {
      if (e.type === "songListChange") {
        const songs = e.data.newSongList;
        const midiDatas = songs.map((s) => {
          return new MIDIData(s);
        });
        this.post({
          type: "sequencerReturn",
          data: {
            type: e.type,
            data: { newSongList: midiDatas },
            id: sequencerID
          },
          currentTime: this.synthesizer.currentSynthTime
        });
        return;
      }
      this.post({
        type: "sequencerReturn",
        data: { ...e, id: sequencerID },
        currentTime: this.synthesizer.currentSynthTime
      });
    };
  }
  postReady(type, data, transferable = []) {
    this.post(
      {
        type: "isFullyInitialized",
        data: {
          type,
          data
        },
        currentTime: this.synthesizer.currentSynthTime
      },
      transferable
    );
  }
  postProgress(type, data) {
    this.post({
      type: "renderingProgress",
      data: {
        type,
        data
      },
      currentTime: this.synthesizer.currentSynthTime
    });
  }
  destroy() {
    this.synthesizer.destroySynthProcessor();
    delete this.synthesizer;
    delete this.sequencer;
  }
  handleMessage(m) {
    const channel = m.channelNumber;
    let channelObject = void 0;
    if (channel >= 0) {
      channelObject = this.synthesizer.midiChannels[channel];
      if (channelObject === void 0) {
        util2.SpessaSynthWarn(
          `Trying to access channel ${channel} which does not exist... ignoring!`
        );
        return;
      }
    }
    switch (m.type) {
      case "midiMessage": {
        this.synthesizer.processMessage(
          m.data.messageData,
          m.data.channelOffset,
          m.data.force,
          m.data.options
        );
        break;
      }
      case "customCcChange": {
        channelObject?.setCustomController(
          m.data.ccNumber,
          m.data.ccValue
        );
        break;
      }
      case "ccReset": {
        if (channel === ALL_CHANNELS_OR_DIFFERENT_ACTION2) {
          this.synthesizer.resetAllControllers();
        } else {
          channelObject?.resetControllers();
        }
        break;
      }
      case "stopAll": {
        if (channel === ALL_CHANNELS_OR_DIFFERENT_ACTION2) {
          this.synthesizer.stopAllChannels(m.data === 1);
        } else {
          channelObject?.stopAllNotes(m.data === 1);
        }
        break;
      }
      case "muteChannel": {
        channelObject?.muteChannel(m.data);
        break;
      }
      case "addNewChannel": {
        this.synthesizer.createMIDIChannel();
        break;
      }
      case "setMasterParameter": {
        this.synthesizer.setMasterParameter(m.data.type, m.data.data);
        break;
      }
      case "setDrums": {
        channelObject?.setDrums(m.data);
        break;
      }
      case "transposeChannel": {
        channelObject?.transposeChannel(m.data.semitones, m.data.force);
        break;
      }
      case "lockController": {
        if (m.data.controllerNumber === ALL_CHANNELS_OR_DIFFERENT_ACTION2) {
          channelObject?.setPresetLock(m.data.isLocked);
        } else {
          if (!channelObject) {
            return;
          }
          channelObject.lockedControllers[m.data.controllerNumber] = m.data.isLocked;
        }
        break;
      }
      case "sequencerSpecific": {
        const seq = this.sequencers[m.data.id];
        if (!seq) {
          return;
        }
        const seqMsg = m.data;
        switch (seqMsg.type) {
          default: {
            break;
          }
          case "loadNewSongList": {
            try {
              const sList = seqMsg.data;
              const songMap = sList.map((s) => {
                if ("duration" in s) {
                  return BasicMIDI2.copyFrom(s);
                }
                return BasicMIDI2.fromArrayBuffer(
                  s.binary,
                  s.fileName
                );
              });
              seq.loadNewSongList(songMap);
            } catch (error) {
              console.error(error);
              this.post({
                type: "sequencerReturn",
                data: {
                  type: "midiError",
                  data: error,
                  id: m.data.id
                },
                currentTime: this.synthesizer.currentSynthTime
              });
            }
            break;
          }
          case "pause": {
            seq.pause();
            break;
          }
          case "play": {
            seq.play();
            break;
          }
          case "setTime": {
            seq.currentTime = seqMsg.data;
            break;
          }
          case "changeMIDIMessageSending": {
            seq.externalMIDIPlayback = seqMsg.data;
            break;
          }
          case "setPlaybackRate": {
            seq.playbackRate = seqMsg.data;
            break;
          }
          case "setLoopCount": {
            seq.loopCount = seqMsg.data;
            break;
          }
          case "changeSong": {
            switch (seqMsg.data.changeType) {
              case songChangeType.shuffleOff: {
                seq.shuffleMode = false;
                break;
              }
              case songChangeType.shuffleOn: {
                seq.shuffleMode = true;
                break;
              }
              case songChangeType.index: {
                if (seqMsg.data.data !== void 0) {
                  seq.songIndex = seqMsg.data.data;
                }
                break;
              }
            }
            break;
          }
          case "getMIDI": {
            if (!seq.midiData) {
              throw new Error("No MIDI is loaded!");
            }
            this.post({
              type: "sequencerReturn",
              data: {
                type: "getMIDI",
                data: seq.midiData,
                id: m.data.id
              },
              currentTime: this.synthesizer.currentSynthTime
            });
            break;
          }
          case "setSkipToFirstNote": {
            seq.skipToFirstNoteOn = seqMsg.data;
            break;
          }
        }
        break;
      }
      case "soundBankManager": {
        try {
          const sfManager = this.synthesizer.soundBankManager;
          const sfManMsg = m.data;
          let font;
          switch (sfManMsg.type) {
            case "addSoundBank": {
              font = SoundBankLoader.fromArrayBuffer(
                sfManMsg.data.soundBankBuffer
              );
              sfManager.addSoundBank(
                font,
                sfManMsg.data.id,
                sfManMsg.data.bankOffset
              );
              this.postReady("soundBankManager", null);
              break;
            }
            case "deleteSoundBank": {
              sfManager.deleteSoundBank(sfManMsg.data);
              this.postReady("soundBankManager", null);
              break;
            }
            case "rearrangeSoundBanks": {
              sfManager.priorityOrder = sfManMsg.data;
              this.postReady("soundBankManager", null);
            }
          }
        } catch (error) {
          this.post({
            type: "soundBankError",
            data: error,
            currentTime: this.synthesizer.currentSynthTime
          });
        }
        break;
      }
      case "keyModifierManager": {
        const kmMsg = m.data;
        const man = this.synthesizer.keyModifierManager;
        switch (kmMsg.type) {
          default: {
            return;
          }
          case "addMapping": {
            man.addMapping(
              kmMsg.data.channel,
              kmMsg.data.midiNote,
              kmMsg.data.mapping
            );
            break;
          }
          case "clearMappings": {
            man.clearMappings();
            break;
          }
          case "deleteMapping": {
            man.deleteMapping(
              kmMsg.data.channel,
              kmMsg.data.midiNote
            );
          }
        }
        break;
      }
      case "requestSynthesizerSnapshot": {
        const snapshot = SynthesizerSnapshot2.create(this.synthesizer);
        this.postReady("synthesizerSnapshot", snapshot);
        break;
      }
      case "requestNewSequencer": {
        this.createNewSequencer();
        break;
      }
      case "setLogLevel": {
        SpessaSynthLogging(
          m.data.enableInfo,
          m.data.enableWarning,
          m.data.enableGroup
        );
        break;
      }
      case "destroyWorklet": {
        this.alive = false;
        this.synthesizer.destroySynthProcessor();
        this.destroy();
        break;
      }
      default: {
        util2.SpessaSynthWarn("Unrecognized event!", m);
        break;
      }
    }
  }
};

// src/synthesizer/worker/write_sf_worker.ts
import { BasicSoundBank } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
async function writeSF2Worker(opts) {
  let sf = this.getBank(opts);
  if (opts.compress && !this.compressionFunction) {
    const e = new Error(
      `Compression enabled but no compression has been provided to WorkerSynthesizerCore.`
    );
    this.post({
      type: "soundBankError",
      data: e,
      currentTime: this.synthesizer.currentSynthTime
    });
    throw e;
  }
  const sq = this.sequencers[opts.sequencerID];
  if (opts.trim) {
    if (!sq.midiData) {
      throw new Error(
        "Sound bank MIDI trimming is enabled but no MIDI is loaded!"
      );
    }
    const sfCopy = BasicSoundBank.copyFrom(sf);
    sfCopy.trimSoundBank(sq.midiData);
    sf = sfCopy;
  }
  let compressionFunction;
  if (this.compressionFunction !== void 0) {
    compressionFunction = (audioData, sampleRate) => this.compressionFunction(
      audioData,
      sampleRate,
      opts.compressionQuality
    );
  }
  const b = await sf.writeSF2({
    ...opts,
    progressFunction: (sampleName, sampleIndex, sampleCount) => {
      this.postProgress("workerSynthWriteFile", {
        sampleCount,
        sampleIndex,
        sampleName
      });
      return new Promise((r) => r());
    },
    compressionFunction
  });
  return {
    binary: b,
    bank: sf
  };
}
async function writeDLSWorker(opts) {
  let sf = this.getBank(opts);
  const sq = this.sequencers[opts.sequencerID];
  if (opts.trim) {
    if (!sq.midiData) {
      throw new Error(
        "Sound bank MIDI trimming is enabled but no MIDI is loaded!"
      );
    }
    const sfCopy = BasicSoundBank.copyFrom(sf);
    sfCopy.trimSoundBank(sq.midiData);
    sf = sfCopy;
  }
  const b = await sf.writeDLS({
    ...opts,
    progressFunction: (sampleName, sampleIndex, sampleCount) => {
      this.postProgress("workerSynthWriteFile", {
        sampleCount,
        sampleIndex,
        sampleName
      });
      return new Promise((r) => r());
    }
  });
  return {
    binary: b,
    bank: sf
  };
}

// src/synthesizer/worker/write_rmi_worker.ts
import { BasicMIDI as BasicMIDI3 } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
async function writeRMIDIWorker(opts) {
  const sq = this.sequencers[opts.sequencerID];
  if (!sq.midiData) {
    throw new Error("No MIDI is loaded!");
  }
  let sf;
  let sfBin;
  if (opts.format === "sf2") {
    const bin = await writeSF2Worker.call(this, opts);
    sfBin = bin.binary;
    sf = bin.bank;
  } else {
    const bin = await writeDLSWorker.call(this, opts);
    sfBin = bin.binary;
    sf = bin.bank;
  }
  const mid = BasicMIDI3.copyFrom(sq.midiData);
  return mid.writeRMIDI(sfBin, {
    soundBank: sf,
    ...opts
  });
}

// src/synthesizer/worker/worker_synthesizer_core.ts
var BLOCK_SIZE2 = 128;
var WorkerSynthesizerCore = class extends BasicSynthesizerCore {
  /**
   * The message port to the playback audio worklet.
   */
  workletMessagePort;
  compressionFunction;
  /**
   * Creates a new worker synthesizer core: the synthesizer that runs in the worker.
   * Most parameters here are provided with the first message that is posted to the worker by the WorkerSynthesizer.
   * @param synthesizerConfiguration The data from the first message sent from WorkerSynthesizer.
   * Listen for the first event and use its data to initialize this class.
   * @param workletMessagePort The first port from the first message sent from WorkerSynthesizer.
   * @param mainThreadCallback postMessage function or similar.
   * @param compressionFunction Optional function for compressing SF3 banks.
   */
  constructor(synthesizerConfiguration, workletMessagePort, mainThreadCallback, compressionFunction) {
    super(
      synthesizerConfiguration.sampleRate,
      {
        enableEventSystem: true,
        enableEffects: true,
        initialTime: synthesizerConfiguration.initialTime
      },
      mainThreadCallback
    );
    this.workletMessagePort = workletMessagePort;
    this.workletMessagePort.onmessage = this.process.bind(this);
    this.compressionFunction = compressionFunction;
    void this.synthesizer.processorInitialized.then(() => {
      this.postReady("sf3Decoder", null);
      this.startAudioLoop();
    });
  }
  /**
   * Handles a message received from the main thread.
   * @param m The message received.
   */
  handleMessage(m) {
    switch (m.type) {
      case "renderAudio": {
        const rendered = renderAudioWorker.call(
          this,
          m.data.sampleRate,
          m.data.options
        );
        const transferable = [];
        for (const r of rendered.effects) transferable.push(r.buffer);
        for (const d of rendered.dry)
          transferable.push(...d.map((c) => c.buffer));
        this.postReady("renderAudio", rendered, transferable);
        break;
      }
      case "writeRMIDI": {
        this.stopAudioLoop();
        void writeRMIDIWorker.call(this, m.data).then((data) => {
          this.postReady(
            "workerSynthWriteFile",
            {
              binary: data,
              fileName: ""
            },
            [data]
          );
          this.startAudioLoop();
        });
        break;
      }
      case "writeSF2": {
        this.stopAudioLoop();
        void writeSF2Worker.call(this, m.data).then((data) => {
          this.postReady(
            "workerSynthWriteFile",
            {
              binary: data.binary,
              fileName: data.bank.soundBankInfo.name + (data.bank.soundBankInfo.version.major === 3 ? ".sf3" : ".sf2")
            },
            [data.binary]
          );
          this.startAudioLoop();
        });
        break;
      }
      case "writeDLS": {
        this.stopAudioLoop();
        void writeDLSWorker.call(this, m.data).then((data) => {
          this.postReady(
            "workerSynthWriteFile",
            {
              binary: data.binary,
              fileName: data.bank.soundBankInfo.name + ".dls"
            },
            [data.binary]
          );
          this.startAudioLoop();
        });
        break;
      }
      default: {
        super.handleMessage(m);
      }
    }
  }
  getBank(opts) {
    const sq = this.sequencers[opts.sequencerID];
    const sf = opts.writeEmbeddedSoundBank && sq.midiData?.embeddedSoundBank ? SoundBankLoader2.fromArrayBuffer(sq.midiData.embeddedSoundBank) : this.synthesizer.soundBankManager.soundBankList.find(
      (b) => b.id === opts.bankID
    )?.soundBank;
    if (!sf) {
      const e = new Error(
        `${opts.bankID} does not exist in the sound bank list!`
      );
      this.post({
        type: "soundBankError",
        data: e,
        currentTime: this.synthesizer.currentSynthTime
      });
      throw e;
    }
    return sf;
  }
  stopAudioLoop() {
    this.synthesizer.stopAllChannels(true);
    for (const seq of this.sequencers) {
      seq.pause();
    }
    this.alive = false;
  }
  startAudioLoop() {
    this.alive = true;
    this.process();
  }
  destroy() {
    this.workletMessagePort.postMessage(null);
    this.stopAudioLoop();
    super.destroy();
  }
  process() {
    if (!this.alive) {
      return;
    }
    const byteStep = BLOCK_SIZE2 * Float32Array.BYTES_PER_ELEMENT;
    const data = new Float32Array(BLOCK_SIZE2 * 34);
    let byteOffset = 0;
    const wetR = new Float32Array(data.buffer, byteOffset, BLOCK_SIZE2);
    byteOffset += byteStep;
    const wetL = new Float32Array(data.buffer, byteOffset, BLOCK_SIZE2);
    byteOffset += byteStep;
    const dry = [];
    for (let i = 0; i < 16; i++) {
      const dryL = new Float32Array(data.buffer, byteOffset, BLOCK_SIZE2);
      byteOffset += byteStep;
      const dryR = new Float32Array(data.buffer, byteOffset, BLOCK_SIZE2);
      byteOffset += byteStep;
      dry.push([dryL, dryR]);
    }
    for (const seq of this.sequencers) {
      seq.processTick();
    }
    this.synthesizer.processSplit(dry, wetL, wetR);
    this.workletMessagePort.postMessage(data, [data.buffer]);
    const t = this.synthesizer.currentSynthTime;
    if (t - this.lastSequencerSync > SEQUENCER_SYNC_INTERVAL) {
      for (let id = 0; id < this.sequencers.length; id++) {
        this.post({
          type: "sequencerReturn",
          data: {
            type: "sync",
            data: this.sequencers[id].currentTime,
            id
          },
          currentTime: t
        });
      }
      this.lastSequencerSync = t;
    }
  }
};

// src/sequencer/sequencer.ts
import {
  ALL_CHANNELS_OR_DIFFERENT_ACTION as ALL_CHANNELS_OR_DIFFERENT_ACTION3,
  BasicMIDI as BasicMIDI4,
  midiMessageTypes as midiMessageTypes2,
  SpessaSynthCoreUtils as SpessaSynthCoreUtils3
} from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";

// src/sequencer/default_sequencer_options.ts
var DEFAULT_SEQUENCER_OPTIONS = {
  skipToFirstNoteOn: true,
  initialPlaybackRate: 1
};

// src/sequencer/seq_event_handler.ts
var SeqEventHandler = class {
  /**
   * The time delay before an event is called.
   * Set to 0 to disable it.
   */
  timeDelay = 0;
  events = {
    songChange: /* @__PURE__ */ new Map(),
    songEnded: /* @__PURE__ */ new Map(),
    metaEvent: /* @__PURE__ */ new Map(),
    timeChange: /* @__PURE__ */ new Map(),
    midiError: /* @__PURE__ */ new Map(),
    textEvent: /* @__PURE__ */ new Map()
  };
  // noinspection JSUnusedGlobalSymbols
  /**
   * Adds a new event listener.
   * @param event The event to listen to.
   * @param id The unique identifier for the event. It can be used to overwrite existing callback with the same ID.
   * @param callback The callback for the event.
   */
  addEvent(event, id, callback) {
    this.events[event].set(id, callback);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Removes an event listener
   * @param name The event to remove a listener from.
   * @param id The unique identifier for the event to remove.
   */
  removeEvent(name, id) {
    this.events[name].delete(id);
  }
  /**
   * Calls the given event.
   * Internal use only.
   * @internal
   */
  callEventInternal(name, eventData) {
    const eventList = this.events[name];
    const callback = () => {
      for (const callback2 of eventList.values()) {
        try {
          callback2(eventData);
        } catch (error) {
          console.error(
            `Error while executing a sequencer event callback for ${name}:`,
            error
          );
        }
      }
    };
    if (this.timeDelay > 0) {
      setTimeout(callback.bind(this), this.timeDelay * 1e3);
    } else {
      callback();
    }
  }
};

// src/sequencer/sequencer.ts
var Sequencer = class {
  /**
   * The current MIDI data, with the exclusion of the embedded sound bank and event data.
   */
  midiData;
  /**
   * The current MIDI data for all songs, like the midiData property.
   */
  songListData = [];
  /**
   * Allows setting up custom event listeners for the sequencer.
   */
  eventHandler = new SeqEventHandler();
  /**
   * Indicates whether the sequencer has finished playing a sequence.
   */
  isFinished = false;
  /**
   * The synthesizer attached to this sequencer.
   */
  synth;
  /**
   * The MIDI port to play to.
   */
  midiOut;
  isLoading = false;
  /**
   * Indicates if the sequencer is paused.
   * Paused if a number, undefined if playing.
   */
  pausedTime = 0;
  getMIDICallback = void 0;
  highResTimeOffset = 0;
  /**
   * Absolute playback startTime, bases on the synth's time.
   */
  absoluteStartTime;
  /**
   * For sending the messages to the correct SpessaSynthSequencer in core
   */
  sequencerID;
  /**
   * Creates a new MIDI sequencer for playing back MIDI files.
   * @param synth synth to send events to.
   * @param options the sequencer's options.
   */
  constructor(synth, options = DEFAULT_SEQUENCER_OPTIONS) {
    this.synth = synth;
    this.absoluteStartTime = this.synth.currentTime;
    this.sequencerID = this.synth.assignNewSequencer(
      this.handleMessage.bind(this)
    );
    this._skipToFirstNoteOn = options?.skipToFirstNoteOn ?? true;
    if (options?.initialPlaybackRate !== 1) {
      this.playbackRate = options?.initialPlaybackRate ?? 1;
    }
    if (!this._skipToFirstNoteOn) {
      this.sendMessage("setSkipToFirstNote", false);
    }
    window.addEventListener(
      "beforeunload",
      this.resetMIDIOutput.bind(this)
    );
  }
  _songIndex = 0;
  /**
   * The current song number in the playlist.
   */
  get songIndex() {
    return this._songIndex;
  }
  /**
   * The current song number in the playlist.
   */
  set songIndex(value) {
    const clamped = Math.max(0, value % this._songsAmount);
    if (clamped === this._songIndex) {
      return;
    }
    this.isLoading = true;
    this.midiData = void 0;
    this.sendMessage("changeSong", {
      changeType: songChangeType.index,
      data: clamped
    });
  }
  _currentTempo = 120;
  /**
   * Current song's tempo in BPM.
   */
  get currentTempo() {
    return this._currentTempo;
  }
  /**
   * The current sequence's length, in seconds.
   */
  get duration() {
    return this.midiData?.duration ?? 0;
  }
  _songsAmount = 0;
  // The amount of songs in the list.
  get songsAmount() {
    return this._songsAmount;
  }
  _skipToFirstNoteOn;
  /**
   * Indicates if the sequencer should skip to first note on.
   */
  get skipToFirstNoteOn() {
    return this._skipToFirstNoteOn;
  }
  /**
   * Indicates if the sequencer should skip to first note on.
   */
  set skipToFirstNoteOn(val) {
    this._skipToFirstNoteOn = val;
    this.sendMessage("setSkipToFirstNote", this._skipToFirstNoteOn);
  }
  /**
   * Internal loop count marker (-1 is infinite).
   */
  _loopCount = -1;
  /**
   * The current remaining number of loops. -1 means infinite looping.
   */
  get loopCount() {
    return this._loopCount;
  }
  /**
   * The current remaining number of loops. -1 means infinite looping.
   */
  set loopCount(val) {
    this._loopCount = val;
    this.sendMessage("setLoopCount", val);
  }
  /**
   * Controls the playback's rate.
   */
  _playbackRate = 1;
  /**
   * Controls the playback's rate.
   */
  get playbackRate() {
    return this._playbackRate;
  }
  /**
   * Controls the playback's rate.
   */
  set playbackRate(value) {
    const t = this.currentTime;
    this.sendMessage("setPlaybackRate", value);
    this.highResTimeOffset *= value / this._playbackRate;
    this._playbackRate = value;
    this.recalculateStartTime(t);
  }
  _shuffleSongs = false;
  /**
   * Indicates if the song order is random.
   */
  get shuffleSongs() {
    return this._shuffleSongs;
  }
  /**
   * Indicates if the song order is random.
   */
  set shuffleSongs(value) {
    this._shuffleSongs = value;
    if (value) {
      this.sendMessage("changeSong", {
        changeType: songChangeType.shuffleOn
      });
    } else {
      this.sendMessage("changeSong", {
        changeType: songChangeType.shuffleOff
      });
    }
  }
  /**
   * Current playback time, in seconds.
   */
  get currentTime() {
    if (this.isLoading) {
      return 0;
    }
    if (this.pausedTime !== void 0) {
      return this.pausedTime;
    }
    return (this.synth.currentTime - this.absoluteStartTime) * this._playbackRate;
  }
  /**
   * Current playback time, in seconds.
   */
  set currentTime(time) {
    this.sendMessage("setTime", time);
  }
  /**
   * A smoothed version of currentTime.
   * Use for visualization as it's not affected by the audioContext stutter.
   */
  get currentHighResolutionTime() {
    if (this.pausedTime !== void 0) {
      return this.pausedTime;
    }
    const highResTimeOffset = this.highResTimeOffset;
    const absoluteStartTime = this.absoluteStartTime;
    const performanceElapsedTime = (performance.now() / 1e3 - absoluteStartTime) * this._playbackRate;
    let currentPerformanceTime = highResTimeOffset + performanceElapsedTime;
    const currentAudioTime = this.currentTime;
    const smoothingFactor = 0.01 * this._playbackRate;
    const timeDifference = currentAudioTime - currentPerformanceTime;
    this.highResTimeOffset += timeDifference * smoothingFactor;
    currentPerformanceTime = this.highResTimeOffset + performanceElapsedTime;
    return currentPerformanceTime;
  }
  /**
   * True if paused, false if playing or stopped.
   */
  get paused() {
    return this.pausedTime !== void 0;
  }
  /**
   * Gets the current MIDI File.
   */
  async getMIDI() {
    return new Promise((resolve) => {
      this.getMIDICallback = resolve;
      this.sendMessage("getMIDI", null);
    });
  }
  /**
   * Loads a new song list.
   * @param midiBuffers The MIDI files to play.
   */
  loadNewSongList(midiBuffers) {
    this.isLoading = true;
    this.midiData = void 0;
    this.sendMessage("loadNewSongList", midiBuffers);
    this._songIndex = 0;
    this._songsAmount = midiBuffers.length;
  }
  /**
   * Connects a given output to the sequencer.
   * @param output The output to connect. Pass undefined to use the connected synthesizer.
   */
  connectMIDIOutput(output) {
    this.resetMIDIOutput();
    this.midiOut = output;
    this.sendMessage("changeMIDIMessageSending", output !== void 0);
    this.currentTime -= 0.1;
  }
  /**
   * Pauses the playback.
   */
  pause() {
    if (this.paused) {
      return;
    }
    this.pausedTime = this.currentTime;
    this.sendMessage("pause", null);
  }
  /**
   * Starts or resumes the playback.
   */
  play() {
    this.recalculateStartTime(this.pausedTime ?? 0);
    this.pausedTime = void 0;
    this.isFinished = false;
    this.sendMessage("play", null);
  }
  handleMessage(m) {
    switch (m.type) {
      case "midiMessage": {
        const midiEventData = m.data.message;
        if (this.midiOut && midiEventData[0] >= 128) {
          this.midiOut.send(midiEventData);
          return;
        }
        break;
      }
      case "songChange": {
        this._songIndex = m.data.songIndex;
        const songChangeData = this.songListData[this._songIndex];
        this.midiData = songChangeData;
        this.isLoading = false;
        this.absoluteStartTime = 0;
        this.callEventInternal("songChange", songChangeData);
        break;
      }
      case "sync": {
        if (Math.abs(m.data - this.currentTime) > 0.05)
          this.recalculateStartTime(m.data);
        break;
      }
      case "timeChange": {
        const time = m.data.newTime;
        this.recalculateStartTime(time);
        this.callEventInternal("timeChange", time);
        break;
      }
      case "pause": {
        this.pausedTime = this.currentTime;
        this.isFinished = m.data.isFinished;
        if (this.isFinished) {
          this.callEventInternal("songEnded", null);
        }
        break;
      }
      case "midiError": {
        this.callEventInternal("midiError", m.data);
        break;
      }
      case "getMIDI": {
        if (this.getMIDICallback) {
          this.getMIDICallback(BasicMIDI4.copyFrom(m.data));
        }
        break;
      }
      case "metaEvent": {
        const event = m.data.event;
        switch (event.statusByte) {
          case midiMessageTypes2.setTempo: {
            this._currentTempo = 6e7 / SpessaSynthCoreUtils3.readBytesAsUintBigEndian(
              event.data,
              3
            );
            break;
          }
          case midiMessageTypes2.text:
          case midiMessageTypes2.lyric:
          case midiMessageTypes2.copyright:
          case midiMessageTypes2.trackName:
          case midiMessageTypes2.marker:
          case midiMessageTypes2.cuePoint:
          case midiMessageTypes2.instrumentName:
          case midiMessageTypes2.programName: {
            if (!this.midiData) {
              break;
            }
            let lyricsIndex = -1;
            if (event.statusByte === midiMessageTypes2.lyric) {
              lyricsIndex = Math.min(
                this.midiData.lyrics.findIndex(
                  (l) => l.ticks === event.ticks
                ),
                this.midiData.lyrics.length - 1
              );
            }
            if (this.midiData.isKaraokeFile && (event.statusByte === midiMessageTypes2.text || event.statusByte === midiMessageTypes2.lyric)) {
              lyricsIndex = Math.min(
                this.midiData.lyrics.findIndex(
                  (l) => l.ticks === event.ticks
                ),
                this.midiData.lyrics.length
              );
            }
            this.callEventInternal("textEvent", {
              event,
              lyricsIndex
            });
            break;
          }
        }
        this.callEventInternal("metaEvent", {
          event: m.data.event,
          trackNumber: m.data.trackIndex
        });
        break;
      }
      case "loopCountChange": {
        this._loopCount = m.data.newCount;
        if (this._loopCount === 0) {
        }
        break;
      }
      case "songListChange": {
        this.songListData = m.data.newSongList.map(
          (m2) => new MIDIData(m2)
        );
        this.midiData = this.songListData[this._songIndex];
        break;
      }
      default: {
        break;
      }
    }
  }
  callEventInternal(type, data) {
    this.eventHandler.callEventInternal(type, data);
  }
  resetMIDIOutput() {
    if (!this.midiOut) {
      return;
    }
    for (let i = 0; i < 16; i++) {
      this.midiOut.send([midiMessageTypes2.controllerChange | i, 120, 0]);
      this.midiOut.send([midiMessageTypes2.controllerChange | i, 123, 0]);
    }
    this.midiOut.send([midiMessageTypes2.reset]);
  }
  recalculateStartTime(time) {
    this.absoluteStartTime = this.synth.currentTime - time / this._playbackRate;
    this.highResTimeOffset = (this.synth.currentTime - performance.now() / 1e3) * this._playbackRate;
    if (this.paused) {
      this.pausedTime = time;
    }
  }
  sendMessage(messageType, messageData) {
    this.synth.post({
      channelNumber: ALL_CHANNELS_OR_DIFFERENT_ACTION3,
      type: "sequencerSpecific",
      data: {
        type: messageType,
        data: messageData,
        id: this.sequencerID
      }
    });
  }
};

// src/utils/buffer_to_wav.ts
import { audioToWav } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
function audioBufferToWav(audioBuffer, options) {
  const channels = [];
  const channelOffset = options?.channelOffset ?? 0;
  const channelCount = options?.channelCount ?? audioBuffer.numberOfChannels;
  for (let i = channelOffset; i < audioBuffer.numberOfChannels; i++) {
    channels.push(audioBuffer.getChannelData(i));
    if (channels.length >= channelCount) {
      break;
    }
  }
  return new Blob([audioToWav(channels, audioBuffer.sampleRate, options)], {
    type: "audio/wav"
  });
}

// src/external_midi/midi_handler.ts
import { SpessaSynthCoreUtils as util3 } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var LibMIDIPort = class {
  port;
  constructor(port) {
    this.port = port;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   *
   */
  get id() {
    return this.port.id;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   *
   */
  get name() {
    return this.port.name;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   *
   */
  get manufacturer() {
    return this.port.manufacturer;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   *
   */
  get version() {
    return this.port.version;
  }
};
var LibMIDIInput = class extends LibMIDIPort {
  connectedSynths = /* @__PURE__ */ new Set();
  constructor(input) {
    super(input);
    input.onmidimessage = (e) => {
      for (const s of this.connectedSynths) {
        if (e.data) s.sendMessage(e.data);
      }
    };
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Connects the input to a given synth, listening for all incoming events.
   * @param synth The synth to connect to.
   */
  connect(synth) {
    this.connectedSynths.add(synth);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Disconnects the input from a given synth.
   * @param synth The synth to disconnect from.
   */
  disconnect(synth) {
    this.connectedSynths.delete(synth);
  }
};
var LibMIDIOutput = class extends LibMIDIPort {
  port;
  constructor(output) {
    super(output);
    this.port = output;
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Connects a given sequencer to the output, playing back the MIDI file to it.
   * @param seq The sequencer to connect.
   */
  connect(seq) {
    seq.connectMIDIOutput(this.port);
  }
  // noinspection JSUnusedGlobalSymbols
  /**
   * Disconnects sequencer from the output, making it play to the attached Synthesizer instead.
   * @param seq The sequencer to disconnect.
   */
  disconnect(seq) {
    seq.connectMIDIOutput(void 0);
  }
};
var MIDIDeviceHandler = class _MIDIDeviceHandler {
  /**
   * The available MIDI inputs. ID maps to the input.
   */
  inputs = /* @__PURE__ */ new Map();
  /**
   * The available MIDI outputs. ID maps to the output.
   */
  outputs = /* @__PURE__ */ new Map();
  constructor(access) {
    for (const [key, value] of access.inputs.entries()) {
      this.inputs.set(key, new LibMIDIInput(value));
    }
    for (const [key, value] of access.outputs.entries()) {
      this.outputs.set(key, new LibMIDIOutput(value));
    }
  }
  /**
   * Attempts to initialize the MIDI Device Handler.
   * @returns The handler.
   * @throws An error if the MIDI Devices fail to initialize.
   */
  static async createMIDIDeviceHandler() {
    if (navigator.requestMIDIAccess) {
      try {
        const response = await navigator.requestMIDIAccess({
          sysex: true,
          software: true
        });
        util3.SpessaSynthInfo(
          "%cMIDI handler created!",
          consoleColors.recognized
        );
        return new _MIDIDeviceHandler(response);
      } catch (error) {
        util3.SpessaSynthWarn(`Could not get MIDI Devices:`, error);
        throw error;
      }
    } else {
      util3.SpessaSynthWarn(
        "Web MIDI API is not supported.",
        consoleColors.unrecognized
      );
      throw new Error("Web MIDI API is not supported.");
    }
  }
};

// src/external_midi/web_midi_link.ts
import { SpessaSynthCoreUtils as SpessaSynthCoreUtils4 } from "https://warbl.xyz/spessasynth/spessasynth_core_4.2.0_index.js";
var WebMIDILinkHandler = class {
  /**
   * Initializes support for Web MIDI Link (https://www.g200kg.com/en/docs/webmidilink/)
   * @param synth The synthesizer to enable support with.
   */
  constructor(synth) {
    window.addEventListener("message", (msg) => {
      if (typeof msg.data !== "string") {
        return;
      }
      const data = msg.data.split(",");
      if (data[0] !== "midi") {
        return;
      }
      data.shift();
      const midiData = data.map((byte) => Number.parseInt(byte, 16));
      synth.sendMessage(midiData);
    });
    SpessaSynthCoreUtils4.SpessaSynthInfo(
      "%cWeb MIDI Link handler created!",
      consoleColors.recognized
    );
  }
};
export {
  DEFAULT_SYNTH_CONFIG,
  MIDIDeviceHandler,
  Sequencer,
  WebMIDILinkHandler,
  WorkerSynthesizer,
  WorkerSynthesizerCore,
  WorkletSynthesizer,
  audioBufferToWav
};
//# sourceMappingURL=index.js.map