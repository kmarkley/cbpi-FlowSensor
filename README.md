## Flow Sensor

### Sensors
##### FlowSensor
* Use any units you prefer: drams, cups, decaliters, barrels, whatever
* Display either accumulated volume or flow rate
    * Flow rate can be expressed in either units/second or units/minute
* Use brew step (below) to calibrate your sensor

##### SimulatedFlowSensor
* Same as above, but reports _simulated_ volume/flow based on power to a chosen actor
* For testing brew step logic

### Brew Steps
##### FlowSensorTransfer
* Turn on one or two actors for transfer
* Transfer either a defined volume or all (i.e. until flow stops)
* Optionally reset sensor volume at beginning and/or end of step

##### FlowSensorCalibrate
* Brew step to calibrate your sensor using your preferred units.
    * Measure volume transferred during step
    * Enter measured volume and reported pulse count in sensor config

##### FlowSensorReset
* Brew step to reset sensor accumulated volume.

---
Under the hood:
* Ultra-lightweight callback function for hall effect to minimize RPi processor load.
* Volume and flow rate are calculated on as-needed basis.
