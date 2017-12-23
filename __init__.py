# -*- coding: utf-8 -*-
# adapted from Flowmeter plugin for Craftbeerpi by nanab
# https://github.com/nanab/Flowmeter
################################################################################

import time
from modules import cbpi
from modules.core.hardware import ActorBase, SensorActive
from modules.core.step import StepBase
from modules.core.props import Property, StepProperty

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
except Exception as e:
    print e

################################################################################
class PulseCounter(object):
    #-------------------------------------------------------------------------------
    def __init__(self, gpio):
        self.count = 0
        self.last = self.prior = 0.0
        try:
            GPIO.setup(gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(gpio, GPIO.RISING, callback=self.pulse_input, bouncetime=20)
            cbpi.app.logger.info("FlowSensor pulse counter initialized for GPIO {}".format(gpio))
        except Exception as e:
            cbpi.notify("Failure to initialize FlowSensor pulse counter", "Could not create callback for GPIO {}".format(gpio), type="danger", timeout="None")
            cbpi.app.logger.error("Failure to initialize FlowSensor pulse counter \n{}".format(e))

    #-------------------------------------------------------------------------------
    def pulse_input(self, channel):
        self.count += 1
        self.prior = self.last
        self.last = time.time()

################################################################################
@cbpi.sensor
class FlowSensor(SensorActive):
    # properties
    a_gpio_prop = Property.Select("GPIO", options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27])
    b_display_prop = Property.Select("Display", options=["volume", "flow"])
    c_volume_units_prop = Property.Text("Volume Units", configurable=True, default_value="L", description="Can by anything, just be sure to calibrate using same units")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/s","/m"])
    e_calibration_units_prop = Property.Number("Calibration Units", configurable=True, default_value=1, description="Actual units transferred during calibration test")
    f_calibration_count_prop = Property.Number("Calibration Count", configurable=True, default_value=485, description="Reported pulse count from calibration test")

    # class attribute!
    _gpio_counters = dict()

    #-------------------------------------------------------------------------------
    @classmethod
    def initialize_gpio_counter(cls, gpio):
        # create a single callback/counter per GPIO (shared by instances)
        if not cls._gpio_counters.get(gpio):
            cls._gpio_counters[gpio] = PulseCounter(gpio)

    #-------------------------------------------------------------------------------
    @classmethod
    def get_gpio_pulse_count(cls, gpio):
        return cls._gpio_counters[gpio].count

    #-------------------------------------------------------------------------------
    @classmethod
    def get_gpio_pulse_rate(cls, gpio):
        # time between last 2 pulses
        delta1 = cls._gpio_counters[gpio].last - cls._gpio_counters[gpio].prior
        # time between last pulse and now
        delta2 = time.time() - cls._gpio_counters[gpio].last
        # inverse of secs between pulses is pulses/sec
        return 1.0/max(delta1, delta2)

    #-------------------------------------------------------------------------------
    def init(self):
        # convert properties to usable attributes
        self.gpio = int(self.a_gpio_prop)
        self.display_type = self.b_display_prop if self.b_display_prop else "volume"
        self.volume_units = self.c_volume_units_prop if self.c_volume_units_prop else "Units"
        self.time_units = str(self.d_time_units_prop)
        self.period_adjust = 60.0 if (self.d_time_units_prop == "/m") else 1.0
        self.calibration = float(self.e_calibration_units_prop)/float(self.f_calibration_count_prop)

        # initialize
        self.initialize_gpio_counter(self.gpio)
        self.reset_volume()
        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def execute(self):
        while self.is_running():
            sensor_data = self.read_sensor_data()
            self.data_received("{:.2f}".format(sensor_data[self.display_type]))
            self.sleep(1.0)

    #-------------------------------------------------------------------------------
    def read_sensor_data(self):

        # pulses since last reset
        pulse_count = self.get_gpio_pulse_count(self.gpio) - self._reset_count
        # volume since last reset
        volume = pulse_count * self.calibration
        # flow rate
        flow = self.get_gpio_pulse_rate(self.gpio) * self.calibration * self.period_adjust

        return {'count':pulse_count, 'flow':flow, 'volume':volume}

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset Volume")
    def reset_volume(self):
        self._reset_count = self.get_gpio_pulse_count(self.gpio)

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.volume_units
        if self.display_type == "flow":
            unit += self.time_units
        return unit

################################################################################
@cbpi.sensor
class SimulatedFlowSensor(SensorActive):
    # properties
    a_flow_actor_prop = Property.Actor("Actor", description="The actor this sensor responds to")
    a_flow_rate_prop = Property.Number("Flow Rate", configurable=True, default_value=2)
    b_display_prop = Property.Select("Display", options=["volume", "flow"])
    c_volume_units_prop = Property.Text("Volume Units", configurable=True, default_value="L", description="Can by anything")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/s","/m"])

    #-------------------------------------------------------------------------------
    def init(self):
        # convert properties to usable attributes
        try:
            self.flow_actor = int(self.a_flow_actor_prop)
            self.flow_rate = float(self.a_flow_rate_prop)
        except:
            self.flow_actor = None
            self.flow_rate = 0.0
        self.display_type = self.b_display_prop if self.b_display_prop else 'volume'
        self.time_units = str(self.d_time_units_prop)
        self.period_adjust = 1.0/60.0 if (self.d_time_units_prop == "/m") else 1.0
        self.volume_units = "Units" if self.c_volume_units_prop == "" else self.c_volume_units_prop
        self.flow_device = None

        # initialize
        self.reset_volume()
        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def execute(self):
        # at startup, wait for actors to initialze
        while cbpi.cache.get("actors") is None:
            self.sleep(5)
        self.flow_device = cbpi.cache.get("actors").get(self.flow_actor, None)

        # primary sensor loop
        while self.is_running():
            sensor_data = self.read_sensor_data()
            self.data_received("{:.2f}".format(sensor_data[self.display_type]))
            self.sleep(1.0)

    #-------------------------------------------------------------------------------
    def read_sensor_data(self):
        time_now = time.time()
        if self.flow_device and int(self.flow_device.state):
            # configured flow rate x actor power level
            flow = self.flow_rate * (float(self.flow_device.power) / 100.0)
            # increment by flow rate x elapsed time
            self._volume += flow * (time_now - self._time_last) * self.period_adjust
        else:
            flow = 0.0
        self._time_last = time_now
        return {'count':0, 'flow':flow, 'volume':self._volume}

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset Volume")
    def reset_volume(self):
        self._volume = 0.0
        self._time_last = time.time()

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.volume_units
        if self.display_type == "flow":
            unit += self.time_units
        return unit

################################################################################
@cbpi.step
class FlowSensorTransfer(StepBase):
    a_sensor_prop = StepProperty.Sensor("Flow Sensor", description="Sensor that contols this step")
    b_actor1_prop = StepProperty.Actor("Actor 1", description="Actor to turn on for the duration of this step")
    c_actor2_prop = StepProperty.Actor("Actor 2", description="Actor to turn on for the duration of this step")
    d_volume_prop = Property.Number("Target Volume", configurable=True, description="Leave blank to continue until flow stops")
    e_reset_start_prop = Property.Select("Reset sensor at start?", options=["Yes","No"])
    f_reset_finish_prop = Property.Select("Reset sensor at finish?", options=["Yes","No"])
    g_threshold_prop = Property.Number("Flow threshold", configurable=True, default_value=0.05, description="Value at which flow is considered stopped")

    #-------------------------------------------------------------------------------
    def init(self):
        # convert properties to usable attributes
        self.sensor = cbpi.cache.get("sensors")[int(self.a_sensor_prop)].instance
        self.actors = [self.b_actor1_prop, self.c_actor2_prop]
        try: self.target_volume = float(self.d_volume_prop)
        except: self.target_volume = 0.0
        self.reset_start = self.e_reset_start_prop == "Yes"
        self.reset_finish = self.f_reset_finish_prop == "Yes"
        try: self.threshold = float(self.g_threshold_prop)
        except: self.threshold = 0.01
        self.flowing = False

        # reset sensor volume if indicated
        if self.reset_start:
            self.sensor.reset_volume()
        # turn on actors
        self.actors_on()

    #-------------------------------------------------------------------------------
    def execute(self):
        sensor_data = self.sensor.read_sensor_data()
        if (not self.flowing) and (sensor_data['flow'] >= self.threshold):
            # flow has started
            self.flowing = True
        elif (self.flowing) and (sensor_data['flow'] <= self.threshold):
            # flow has stopped
            self.next()
        elif self.target_volume and (sensor_data['volume'] >= self.target_volume):
            # target volume reached
            self.next()

    #-------------------------------------------------------------------------------
    def finish(self):
        # turn actors off
        self.actors_off()

        # notify complete and total volume
        sensor_data = self.sensor.read_sensor_data()
        self.notify("{} complete".format(self.name), "Total Volume: {:.2f}{}".format(sensor_data['volume'], self.sensor.volume_units), timeout=None)

        # reset sensor volume if indicated
        if self.reset_finish:
            self.sensor.reset_volume()

    #-------------------------------------------------------------------------------
    def reset(self):
        self.actors_off()

    #-------------------------------------------------------------------------------
    def actors_on(self):
        for actor in self.actors:
            try: self.actor_on(int(actor))
            except: pass

    def actors_off(self):
        for actor in self.actors:
            try: self.actor_off(int(actor))
            except: pass

################################################################################
@cbpi.step
class FlowSensorCalibrate(StepBase):
    # properties
    actor_prop = StepProperty.Actor("Actor")
    sensor_prop = StepProperty.Sensor("Sensor")
    timer_prop = Property.Number("Timer", configurable=True, default_value=10)
    threshold_prop = Property.Number("Flow threshold", configurable=True, default_value=0.05,  description="Value at which flow is considered stopped")

    #-------------------------------------------------------------------------------
    def init(self):
        # convert properties to usable attributes
        self.actor = int(self.actor_prop)
        self.sensor = cbpi.cache.get("sensors")[int(self.sensor_prop)].instance
        self.threshold = float(self.threshold_prop)
        self.flowing = False

        # reset sensor volume to start calibration
        self.sensor.reset_volume()
        # turn on actor
        self.actor_on(self.actor)
        # start timer
        if self.is_timer_finished() is None:
            self.start_timer(float(self.timer_prop) * 60)

    #-------------------------------------------------------------------------------
    def execute(self):
        data = self.sensor.read_sensor_data()
        if (not self.flowing) and (data['flow'] >= self.threshold):
            # flow has started
            self.flowing = True
        elif (self.flowing) and (data['flow'] <= self.threshold):
            # flow has stopped
            self.next()
        elif self.is_timer_finished() == True:
            # timer has expired
            self.next()

    #-------------------------------------------------------------------------------
    def finish(self):
        # turn off actor
        self.actor_off(self.actor)

        # notify complete and total pulses
        sensor_data = self.sensor.read_sensor_data()
        self.notify("Flow Sensor Calibration Complete", self.sensor.name, type="success", timeout=None)
        self.notify("Pulse Count: {}".format(sensor_data['count']), self.sensor.name, type="info", timeout=None)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.actor_off(self.actor)

################################################################################
@cbpi.step
class FlowSensorReset(StepBase):
    sensor_prop = StepProperty.Sensor("Sensor")

    #-------------------------------------------------------------------------------
    def init(self):
        self.sensor = cbpi.cache.get("sensors")[int(self.sensor_prop)].instance
        self.sensor.reset_volume()
        self.next()
