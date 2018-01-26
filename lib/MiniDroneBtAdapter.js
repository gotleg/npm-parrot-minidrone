const Logger = require('winston');
//Logger.level = 'debug'
const EventEmitter = require('events');

// MiniDrone Command classes and class methods
// https://github.com/Parrot-Developers/libARCommands/blob/master/Xml/MiniDrone_commands.xml
const MD_CLASSES = {
    PILOTING: 0x00,
    SPEED_SETTINGS: 0x01,
    ANIMATION: 0x04,
    MEDIA_RECORD: 0x06,
    PILOTING_SETTINGS: 0x08,
    NAVIGATION_DATA_STATE: 0x18,
};
const MD_METHODS = {
    TRIM: 0x00,
    TAKEOFF: 0x01,
    LAND: 0x03,
    EMERGENCY: 0x04,
    PICTURE: 0x01,
    FLIP: 0x00,
    MAX_ALTITUDE: 0x00,
    MAX_TILT: 0x01,
    MAX_VERTICAL_SPEED: 0x00,
    MAX_ROTATION_SPEED: 0x01,
    DRONE_POSITION: 0x00,
    DRONE_SPEED: 0x01,
};
const MD_DATA_TYPES = {
    ACK: 0x01,
    DATA: 0x02,
    LLD: 0x03,
    DATA_WITH_ACK: 0x04,
};
const animations = {
            flipFront: 0x00,
            flipBack: 0x01,
            flipRight: 0x02,
            flipLeft: 0x03,
        };
const pictureState = ['ready', 'busy', 'notAvailable']
const pictureStateError = ['ok', 'unknown', 'cameraKO', 'memoryFull', 'lowBattery']

// BTLE Characteristic keys
const RX_COMMAND_WITH_ACK   = 'fb0e';   // fb : Receive commands    - 0e : ACK_DRONE_DATA       # drone data that needs an ack (needs to be ack on 1e)
const RX_COMMAND_NO_ACK     = 'fb0f';   // fb : Receive commands    - 0f : NO_ACK_DRONE_DATA    # data from drone (including battery and others), no ack
                                        // fb : Receive commands    - 1b : ACK_COMMAND_SENT     # ack 0b channel, SEND_WITH_ACK
                                        // fb : Receive commands    - 1c : ACK_HIGH_PRIORITY    # ack 0c channel, SEND_HIGH_PRIORITY

const FLIGHT_PARAMS_KEY     = 'fa0a';   // fa : Send commands       - 0a : SEND_NO_ACK          # not-ack commandsandsensors (PCMD only)
const COMMAND_KEY           = 'fa0b';   // fa : Send commands       - 0b : SEND_WITH_ACK        # ack commandsandsensors (all piloting commandsandsensors)
const EMERGENCY_KEY         = 'fa0c';   // fa : Send commands       - 0c : SEND_HIGH_PRIORITY   # emergency commandsandsensors
                                        // fa : Send commands       - 1e : ACK_COMMAND          # ack for data sent on 0e

// TODO: need all these?
const CHARACTERISTIC_MAP = [
    RX_COMMAND_NO_ACK, RX_COMMAND_WITH_ACK, 'fb1b', 'fb1c', 'fd22', 'fd23', 'fd24', 'fd52', 'fd53', 'fd54',
];

// Drone IDs
const MANUFACTURER_SERIALS = ['4300cf1900090100', '4300cf1909090100', '4300cf1907090100'];
const DRONE_PREFIXES = ['RS_', 'Mars_', 'Travis_', 'Maclan_', 'Mambo_', 'Blaze_', 'NewZ_'];

const MD_DEVICE_TYPE = 0x02;

const FLIGHT_STATUSES = ['landed', 'taking off', 'hovering', 'flying',
                         'landing', 'emergency', 'rolling', 'initializing'];

const ALERT_STATE_CHANGED = ['none', 'user', 'cut_out', 'critical_battery', 'low_battery'];

/**
 * Network adapter between drone and Noble BTLE
 * Abstracts away all the characteristics, buffers
 * and steps bullshit.
 *
 * @author Christopher Fetherston <chris@cfetherston.com>
 */
class MiniDroneBtAdapter extends EventEmitter {
    /**
     * Instantiates a new instance of the MiniDroneBtAdapter class
     *
     * @param {Object} options Configuration options object
     * @param {String} options.droneFilter The name of the drone to restrict connection to
     */
    constructor(options) {
        super();
        const defaults = {
            droneFilter: '',
        };
        this.options = Object.assign({}, defaults, options);
        // noble is not a constructor
        this.noble = require('noble');
        this.connected = false;
        this.peripheral = null;
        this.characteristics = [];
        this.batteryLevel = 'Unknown';
        // Steps hold the command sequence, they increment with every new characteristic write
        // and should be reset once reaching 255
        this.steps = {};
        this.steps[FLIGHT_PARAMS_KEY] = 0;
        this.steps[COMMAND_KEY] = 0;
        this.steps[EMERGENCY_KEY] = 0;
        this.flightStatus = null;
        this.alertStatus = 0;
        // flight param cache to only send values that have changed
        this.flightParams = {
            roll: 0,
            pitch: 0,
            yaw: 0,
            altitude: 0,
        };

        // bind noble event handlers
        this.noble.on('stateChange', (state) => this.onNobleStateChange(state));
        this.noble.on('discover', (peripheral) => this.onPeripheralDiscovery(peripheral));
        this.noble.on('scanStart', () => { Logger.debug('Noble scan started')})
        this.noble.on('scanStop', () => { Logger.debug('Noble scan stopped')})
        this.noble.on('warning', (message) => { Logger.debug('Noble warning : ' + message)})
    }

    /**
     * Event handler for when noble broadcasts a state change
     * @param  {String} state a string describing noble's state
     * @return {undefined}
     */
    onNobleStateChange(state) {
        if (state === 'poweredOn') {
            Logger.info(`Bluetooth dongle ready, start scanning`);
            this.noble.startScanning();
        } else if (state === 'poweredOff') {
          this.emit('poweredOff', true);
        }
    }

    /**
     * Writes a buffer to a BTLE peripheral characteristic
     * Most convince methods in this class point to this method
     *
     * @param  {String} uuid   the characteristic's UUID
     * @param  {Buffer} buffer stream of binary data
     * @return {undefined}
     */
    write(uuid, buffer) {
        if (!this.characteristics.length) {
            Logger.warn('You must have bluetooth enabled and be connected to a drone before executing a command.');
           return;
        }

        // Sequence number can only be stored in one byte, so we must reset after 255
        if (this.steps[uuid] >= 255) {
            this.steps[uuid] = 0;
        }

        this.getCharacteristic(uuid).write(buffer, true);
    }

    /**
     * Creates a buffer with the common values needed to write to the drone
     * @param  {String} uuid The characteristic UUID
     * @param  {Array}  args The buffer arguments, usually the above command constants
     * @return {buffer}      A freshly created Buffer stream
     */
    createBuffer(uuid, args = []) {
        const buffArray = [MD_DATA_TYPES.DATA, ++this.steps[uuid] & 0xFF, MD_DEVICE_TYPE];
        return new Buffer(buffArray.concat(args));
    }

    /**
     * Writes the drones roll, pitch, yaw and altitude to the device
     * TODO: This could be smarter and cache values and only update when changed
     *
     * @param  {object} flightParams Object containing any roll, pitch, yaw and altitude
     * @return {undefined}
     */
    writeFlightParams(flightParams) {
        const buffer = new Buffer(19);
        this.flightParams = flightParams;

        buffer.fill(0);
        buffer.writeInt16LE(2, 0);                                  //dataType (1:ACK,2:Data_NO_ACK,3:LOW_LATENCY_DATA,4:DATA_WITH_ACK)
        buffer.writeInt16LE(++this.steps[FLIGHT_PARAMS_KEY], 1);    //step
        buffer.writeInt16LE(2, 2);                                  //project (0:COMMON,1:ARDRONE3,2:MINIDRONE)
        buffer.writeInt16LE(0, 3);                                  //class (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(2, 4);                                  //command (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(0, 5);                                  //empty ?
        buffer.writeInt16LE(1, 6); // roll and pitch bool           //Argument 1
        buffer.writeInt16LE(this.flightParams.roll, 7);             //Argument 2
        buffer.writeInt16LE(this.flightParams.pitch, 8);            //Argument 3
        buffer.writeInt16LE(this.flightParams.yaw, 9);              //Argument 4
        buffer.writeInt16LE(this.flightParams.altitude, 10);        //Argument 5
        buffer.writeFloatLE(0, 11);                                 //Argument 6 (Timestamp)

        this.write(FLIGHT_PARAMS_KEY, buffer);
        this.emit('flightParamChange', this.flightParams);
    }

    /**
     * Convenience method for writing the flat trim command
     * @return {undefined}
     */
    writeTrim() {
        const buffer = this.createBuffer(COMMAND_KEY, [MD_CLASSES.PILOTING, MD_METHODS.TRIM, 0x00]);
        this.write(COMMAND_KEY, buffer);
        Logger.info('Trim command called');
    }

    /**
     * Convenience method for writing the takeoff command
     * @return {undefined}
     */
    writeTakeoff() {
        const buffer = this.createBuffer(COMMAND_KEY, [MD_CLASSES.PILOTING, MD_METHODS.TAKEOFF, 0x00]);
        this.write(COMMAND_KEY, buffer);
        Logger.info('Takeoff command called');
    }

    /**
     * Convenience method for writing the land command
     * @return {undefined}
     */
    writeLand() {
        const buffer = this.createBuffer(COMMAND_KEY, [MD_CLASSES.PILOTING, MD_METHODS.LAND, 0x00]);
        this.write(COMMAND_KEY, buffer);
        Logger.info('Land command called');
    }

    /**
     * Convenience method for writing the emergency command
     * @return {undefined}
     */
    writeEmergency() {
        const buffer = this.createBuffer(EMERGENCY_KEY, [MD_CLASSES.PILOTING, MD_METHODS.EMERGENCY, 0x00]);
        this.write(EMERGENCY_KEY, buffer);
        Logger.info('Emergency command called');
    }

    /**
     * Convenience method for writing the media take a picture command
     * @return {undefined}
     */
    writeTakePicture() {
        const buffer = this.createBuffer(COMMAND_KEY, [MD_CLASSES.MEDIA_RECORD, MD_METHODS.PICTURE, 0x00]);
        this.write(COMMAND_KEY, buffer);
        Logger.info('Take picture command called');
    }

    /**
     * Convenience method for writing animation class methods
     * @param {String} animation The animation direction
     * @return {undefined}
     */
    writeAnimation(animation) {
        if (typeof animations[animation] === 'undefined') {
            return;
        }
        // this one is a little weird, don't understand the extra
        // argument after the flip class constant ¯\_(ツ)_/¯
        const buffer = this.createBuffer(COMMAND_KEY, [MD_CLASSES.ANIMATION, MD_METHODS.FLIP, 0x00, animations[animation], 0x00, 0x00, 0x00]);
        this.write(COMMAND_KEY, buffer);
        Logger.info(`Animation command called with ${animation} argument`);
    }

    /**
     * Convenience method for setting the drone's altitude limitation
     * @param  {Integer} altitude the altitude in meters (2m-10m for Airborne Cargo / 2m - 25m for Mambo)
     * @return {undefined}
     */
    writeMaxAltitude(altitude) {
        const buffer = new Buffer(19);
        buffer.fill(0);
        buffer.writeInt16LE(2, 0);                                     //dataType (1:ACK,2:Data_NO_ACK,3:LOW_LATENCY_DATA,4:DATA_WITH_ACK)
        buffer.writeInt16LE(++this.steps[FLIGHT_PARAMS_KEY], 1);       //step
        buffer.writeInt16LE(2, 2);                                     //project (0:COMMON,1:ARDRONE3,2:MINIDRONE)
        buffer.writeInt16LE(MD_CLASSES.PILOTING_SETTINGS, 3);          //class (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(MD_METHODS.MAX_ALTITUDE, 4);               //command (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(0, 5);                                     //empty ?
        buffer.writeFloatLE(altitude, 6); // altitude               //Argument 1
        buffer.writeFloatLE(0, 10);                                 //Argument 2 (Timestamp)

        this.write(COMMAND_KEY, buffer);
        this.emit('maxAltitudeChange', altitude);
        Logger.debug(`Setting max altitude to ${altitude}m`);
    }

    /**
     * Convenience method for setting the drone's max tilt limitation
     * @param  {integer} tilt The max tilt from 0-100 (0 = 5° - 100 = 20°)
     * @return {undefined}
     */
    writeMaxTilt(tilt) {
        const buffer = new Buffer(19);

        buffer.fill(0);
        buffer.writeInt16LE(2, 0);                                     //dataType (1:ACK,2:Data_NO_ACK,3:LOW_LATENCY_DATA,4:DATA_WITH_ACK)
        buffer.writeInt16LE(++this.steps[FLIGHT_PARAMS_KEY], 1);       //step
        buffer.writeInt16LE(2, 2);                                     //project (0:COMMON,1:ARDRONE3,2:MINIDRONE)
        buffer.writeInt16LE(MD_CLASSES.PILOTING_SETTINGS, 3);          //class (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(MD_METHODS.MAX_TILT, 4);                   //command (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(0, 5);                                     //empty ?
        buffer.writeFloatLE(tilt, 6); // tilt                       //Argument 1
        buffer.writeFloatLE(0, 10);                                 //Argument 2 (Timestamp)

        this.write(COMMAND_KEY, buffer);

        this.emit('maxTiltChange', tilt);
        Logger.debug(`Setting max tilt to ${tilt}% (20° max)`);
    }

    /**
     * Convenience method for setting the drone's max vertical speed limitation
     * @param  {float} verticalSpeed The max vertical speed from 0.5m/s - 2m/s
     * @return {undefined}
     */
    writeMaxVerticalSpeed(verticalSpeed) {
        const buffer = new Buffer(19);

        buffer.fill(0);
        buffer.writeInt16LE(2, 0);                                     //dataType (1:ACK,2:Data_NO_ACK,3:LOW_LATENCY_DATA,4:DATA_WITH_ACK)
        buffer.writeInt16LE(++this.steps[FLIGHT_PARAMS_KEY], 1);       //step
        buffer.writeInt16LE(2, 2);                                     //project (0:COMMON,1:ARDRONE3,2:MINIDRONE)
        buffer.writeInt16LE(MD_CLASSES.SPEED_SETTINGS, 3);             //class (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(MD_METHODS.MAX_VERTICAL_SPEED, 4);         //command (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(0, 5);                                     //empty ?
        buffer.writeFloatLE(verticalSpeed, 6); // speed             //Argument 1
        buffer.writeFloatLE(0, 10);                                 //Argument 2 (Timestamp)

        this.write(COMMAND_KEY, buffer);

        this.emit('maxVerticalSpeedChange', verticalSpeed);
        Logger.debug(`Setting max vertical speed to ${verticalSpeed} m/s`);
    }

    /**
     * Convenience method for setting the drone's max rotation speed limitation
     * @param  {float} rotationSpeed The max rotation speed from (50°-360° for Airborne Cargo / 50° - 180° for Mambo)
     * @return {undefined}
     */
    writeMaxRotationSpeed(rotationSpeed) {
        const buffer = new Buffer(19);

        buffer.fill(0);
        buffer.writeInt16LE(2, 0);                                     //dataType (1:ACK,2:Data_NO_ACK,3:LOW_LATENCY_DATA,4:DATA_WITH_ACK)
        buffer.writeInt16LE(++this.steps[FLIGHT_PARAMS_KEY], 1);       //step
        buffer.writeInt16LE(2, 2);                                     //project (0:COMMON,1:ARDRONE3,2:MINIDRONE)
        buffer.writeInt16LE(MD_CLASSES.SPEED_SETTINGS, 3);             //class (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(MD_METHODS.MAX_ROTATION_SPEED, 4);         //command (see:https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml)
        buffer.writeInt16LE(0, 5);                                     //empty ?
        buffer.writeFloatLE(rotationSpeed, 6); // rotationSpeed     //Argument 1
        buffer.writeFloatLE(0, 10);                                 //Argument 2 (Timestamp)

        this.write(COMMAND_KEY, buffer);

        this.emit('maxRotationSpeedChange', rotationSpeed);
        Logger.debug(`Setting max rotation speed to ${rotationSpeed} °/s`);
    }

    /**
     * Event handler for when noble discovers a peripheral
     * Validates it is a drone and attempts to connect.
     *
     * @param {Peripheral} peripheral a noble peripheral class
     * @return {undefined}
     */
    onPeripheralDiscovery(peripheral) {
        if (!this.validatePeripheral(peripheral)) {
            return;
        }
        Logger.info(`Peripheral ${peripheral.advertisement.localName} found, trying to connect...`);
        this.noble.stopScanning();

        this.peripheral = peripheral;

        this.peripheral.connect((error) => {
            if (error) {
                throw error;
            }
            this.setupPeripheral();
            this.peripheral.once('disconnect', () => this.onDisconnect());
        });
    }

    /**
     * Event handler for when noble disconnect from a peripheral
     * Set connected state to false and start scanning
     * @return {undefined}
     */
    onDisconnect() {
        if (this.connected) {
            Logger.info('Disconnected from drone');
        }
        this.characteristics = [];
        this.peripheral = null;
        this.connected = false;
        this.emit('disconnected');
        Logger.info("Start scanning")
        this.noble.startScanning();
    }

    disconnect() {
      this.peripheral.disconnect(function(error) {
         Logger.info('disconnected from peripheral: ' + peripheral.uuid);
      });
    }

    /**
     * Sets up a peripheral and finds all of it's services and characteristics
     * @return {undefined}
     */
    setupPeripheral() {
        if (!this.peripheral) {
            return;
        }
        this.peripheral.discoverAllServicesAndCharacteristics((err, services, characteristics) => {
            if (err) {
                throw err;
            }
            this.characteristics = characteristics;
            
            // subscribe to these keys
            CHARACTERISTIC_MAP.forEach((key) => {
                this.getCharacteristic(key).subscribe();
            });

            // Register listener for notifications without ack.
            this.getCharacteristic(RX_COMMAND_NO_ACK).on('data', (data, isNotification) => {
                this.onRXCommandNoACK(data, isNotification);
            });

            // Register a listener for flight status changes
            this.getCharacteristic(RX_COMMAND_WITH_ACK).on('data', (data, isNotification) => {
                this.onRXCommandWithACK(data, isNotification);
            });

            this.connected = true;
            Logger.info(`Device connected ${this.peripheral.advertisement.localName}`);

            // I don't know why this needs some time
            setTimeout(() => this.emit('connected'), 200);
        });
    }

    /**
     * Updates Rssi to get signal strength
     */
    updateRssi() {
        if (!this.peripheral) {
            return;
        }
        this.peripheral.updateRssi((error, rssi) => {
            if (!error) {
                this.emit('rssiUpdate', rssi);
            }
        });
    }

    /**
     * Validates a noble Peripheral class is a Parrot MiniDrone
     * @param {Peripheral} peripheral a noble peripheral object class
     * @return {boolean} If the peripheral is a drone
     */
    validatePeripheral(peripheral) {
        if (!peripheral) {
            return false;
        }
        const localName = peripheral.advertisement.localName;
        const manufacturer = peripheral.advertisement.manufacturerData;

        var re = new RegExp(this.options.droneFilter);
        const localNameMatch = (this.options.droneFilter == '' ? localName && DRONE_PREFIXES.some((prefix) => localName.indexOf(prefix) >= 0):re.test(localName));

        const manufacturerMatch = manufacturer && (MANUFACTURER_SERIALS.indexOf(manufacturer) >= 0);

          // Is TRUE according to droneFilter or if empty, for EITHER an "RS_" name OR manufacturer code.
        return localNameMatch || manufacturerMatch;
    }

    /**
     * Finds a Noble Characteristic class for the given characteristic UUID
     * @param {String} uuid The characteristics UUID
     * @return {Characteristic} The Noble Characteristic corresponding to that UUID
     */
    getCharacteristic(uuid) {
        if (!this.characteristics.length) {
            Logger.warn('BTLE Device must be connected before calling this method');
            return false;
        }
        return this.characteristics.filter((c) => c.uuid.search(new RegExp(uuid)) !== -1)[0];
    }

    /**
     * Event handler for when the drone broadcasts a flight status change
     * @param {Object} data The event data
     * @param {Boolean} isNotification If the broadcast event is a notification
     * @return {undefined}
     */
    onRXCommandWithACK(data, isNotification) {
        this.decodePacket(data);
    }

    /**
     * Event handler for when the drone broadcasts a batter status change
     * @param {Object} data he event data
     * @param {Boolean} isNotification If the broadcast event is a notification
     * @return {undefined}
     */
    onRXCommandNoACK(data, isNotification) {
        this.decodePacket(data);
    }

    decodePacket(data) {     
        var dataType = data.readUInt8(0);
        /*
        1 : Acknoledgment of previously received data
        2 : Data (no ack requested)
        3 : Low latency data
        4 : Data with ack : Data requesting an ack
        */
        var steps = data.readUInt8(1);
        /* counter for each interaction */ 
        var project = data.readUInt8(2);
        /*
        0 : COMMON
        1 : ARDRONE3
        2 : MINIDRONE
        */
        var msgClass = data.readUInt8(3);
        /* see https://github.com/Parrot-Developers/arsdk-xml/tree/master/xml
        for class (common.xml, ardrone3.xml, minidrone.xml)
        */
        var commandKey = data.readUInt8(4);
        /* see https://github.com/Parrot-Developers/arsdk-xml/tree/master/xml
        for commands (common.xml, ardrone3.xml, minidrone.xml)
        */
        switch (dataType)
        {
            case 1:
                //Logger.debug("ACK");
            break;
            case 2:
                //Logger.debug("RX DATA NO ACK");
            break;
            case 3:
                //Logger.debug("LOW LATENCY DATA");
            break;
            case 4:
                //Logger.debug("DATA WITH ACK");
            break;
            default:
                Logger.info("DataType missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
            break;
        }
        switch (project)
        {
            case 0:
                Logger.debug("COMMON");
                switch (msgClass) {
                    case 5:        // CommonState
                        switch (commandKey)
                        {
                           case 1: // BatteryStateChanged
                                this.batteryLevel = data.readUInt8(6);
                                this.emit('batteryStatusChange', this.batteryLevel);
                                Logger.debug(`Battery level: ${this.batteryLevel}%`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    case 30:        // RunState
                        switch (commandKey)
                        {
                           case 0: // RunIdChanged
                                Logger.debug("RunIdChanged : Used for Parrot Academy, no need to implement. Send a new unique number at TakeOff");
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    default:
                        Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                    break;
                }
            break;
            case 1:
                Logger.info("ARDRONE3");
            break;
            case 2:
                Logger.debug("MINIDRONE");
                switch (msgClass) {
                    case 18:        // NavigationDataState
                        switch (commandKey)
                        {
                            case 0: // Drone Position
                                var posx = data.readFloatLE(6);
                                var posy = data.readFloatLE(10);
                                var posz = data.readUInt16LE(14);
                                var psi = data.readUInt16LE(16);
                                var ts = data.readUInt16LE(18);
                                var dronePosition = {
                                    x: posx,
                                    y: posy,
                                    z: posz,
                                    psi: psi,
                                    ts: ts,
                                };
                                this.emit('dronePositionChange', dronePosition);
                                Logger.debug(`Position : x=${posx} - y=${posy} - z=${posz} - psi=${psi} - timestamp=${ts}`);
                            break;
                            case 1: // DroneSpeed
                                var speedx = data.readFloatLE(6);
                                var speedy = data.readFloatLE(10);
                                var speedz = data.readFloatLE(14);
                                var ts = data.readUInt16LE(18);
                                var droneSpeed = {
                                    x: speedx,
                                    y: speedy,
                                    z: speedz,
                                    ts: ts,
                                };
                                this.emit('droneSpeedChange', droneSpeed);
                                Logger.debug(`Speed : x=${speedx} - y=${speedy} - z=${speedz} - timestamp=${ts}`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    case 3:         // Piloting state
                        switch (commandKey)
                        {
                            case 1: // FlyingStateChanged
                                this.flightStatus = FLIGHT_STATUSES[data.readUInt8(6)];
                                this.emit('flightStatusChange', this.flightStatus);
                                Logger.debug(`Flight status = ${this.flightStatus} - ${data.readUInt8(6)}`);
                            break;
                            case 2: // AlertStateChanged
                                this.alertStatus = ALERT_STATE_CHANGED[data.readUInt8(6)];
                                this.emit('alertStateChange', this.alertStatus);
                                Logger.debug(`AlertStateChange = ${this.alertStatus} - ${data.readUInt8(6)}`);

                            break;
                            case 3: // AutoTakeOffModeChanged
                                var autoTakeOffModeChanged = data.readUInt8(6);
                                Logger.debug(`AutoTakeOffModeChanged = ${autoTakeOffModeChanged}`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    case 5:         // SpeedSettingsState
                        switch (commandKey)
                        {
                            case 0: // MaxVerticalSpeedChanged
                                Logger.info(`MaxVerticalSpeedChanged : current=${data.readFloatLE(6)} - min=${data.readFloatLE(10)} - max=${data.readFloatLE(14)}`);
                            break;
                            case 1: // MaxRotationSpeedChanged
                                Logger.info(`MaxRotationSpeedChanged : current=${data.readFloatLE(6)} - min=${data.readFloatLE(10)} - max=${data.readFloatLE(14)}`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    case 7:         // MediaRecordState
                        switch (commandKey)
                        {
                            case 1: // PictureStateChangedV2
                                Logger.debug(`PictureStateChangedV2 : state= ${pictureState[data.readUInt8(6)]} - error=${pictureStateError[data.readUInt8(7)]}`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    case 9:         // PilotingSettingsState
                        switch (commandKey)
                        {
                            case 0: // MaxAltitudeChanged
                                Logger.info(`MaxAltitudeChanged : current=${data.readFloatLE(6)} - min=${data.readFloatLE(10)} - max=${data.readFloatLE(14)}`);
                            break;
                            case 1: // MaxTiltChanged
                                Logger.info(`MaxTiltChanged : current=${data.readFloatLE(6)} - min=${data.readFloatLE(10)} - max=${data.readFloatLE(14)}`);
                            break;
                            default:
                                Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                            break;
                        }
                    break;
                    default:
                        Logger.info("Packet implementation missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
                    break;
                }
            break;
            default:
                Logger.info("Project missing missing : dataType:" + dataType + " | project:" + project + " | class:" +  msgClass + " | command:" +  commandKey);
            break;
        }
    }
}

module.exports = MiniDroneBtAdapter;
