# -*- coding: utf-8 -*-
# adapted from Flowmeter plugin for Craftbeerpi by nanab
# https://github.com/nanab/Flowmeter
################################################################################

import time
from modules import cbpi
from modules.core.hardware import ActorBase, SensorActive
from modules.core.step import StepBase
from modules.core.props import Property, StepProperty

################################################################################

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
except Exception as e:
    print e

################################################################################
class PulseCounter(object):
    #-------------------------------------------------------------------------------
    def __init__(self, gpio):
        self.pulse_count = 0
        try:
            GPIO.setup(gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(gpio, GPIO.RISING, callback=self.pulse_input, bouncetime=20)
            cbpi.app.logger.info("FlowSensor pulse counter initialized for GPIO {}".format(gpio))
        except Exception as e:
            cbpi.app.logger.error("Failure to initialize FlowSensor pulse counter \n{}".format(e))
    #-------------------------------------------------------------------------------
    def pulse_input(self, channel):
        self.pulse_count += 1


################################################################################
@cbpi.sensor
class FlowSensor(SensorActive):
    a_gpio_prop = Property.Select("GPIO", options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27])
    b_display_prop = Property.Select("Display", options=["Volume", "Flow Rate"])
    c_volume_units_prop = Property.Text("Volume Units", configurable=True, default_value="L", description="Can by anything, just be sure to calibrate using same units")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/s","/m"])
    e_calibration_units_prop = Property.Number("Calibration Units", configurable=True, default_value=1, description="Actual units transferred during calibration test")
    f_calibration_count_prop = Property.Number("Calibration Count", configurable=True, default_value=485, description="Reported pulse count from calibration test")

    counter = dict()

    #-------------------------------------------------------------------------------
    def init(self):
        self.gpio = int(self.a_gpio_prop)
        self.flow_display = self.b_display_prop == "Flow Rate"
        self.volume_units = "Units" if self.c_volume_units_prop == "" else self.c_volume_units_prop
        self.time_units = str(self.d_time_units_prop)
        self.period_adjust = 60.0 if (self.d_time_units_prop == "/m") else 1.0
        self.calibration = float(self.e_calibration_units_prop)/float(self.f_calibration_count_prop)

        if not self.counter.get(self.gpio):
            self.counter[self.gpio] = PulseCounter(self.gpio)

        self.reset_count = self.counter[self.gpio].pulse_count
        self.last_count = self.reset_count
        self.last_time = time.time()
        self.volume = 0.0
        self.flow = 0.0

        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def update_values(self):
        now = time.time()
        pulse_count = self.counter[self.gpio].pulse_count - self.reset_count
        count_delta = pulse_count - self.last_count
        self.volume = pulse_count * self.calibration
        self.flow = (count_delta * self.calibration)/(now - self.last_time) * self.period_adjust
        self.last_count = pulse_count
        self.last_time = now

    #-------------------------------------------------------------------------------
    def execute(self):
        while self.is_running():
            # an active brew step may have already updated values
            if time.time() - self.last_time >= 1.0:
                self.update_values()
            value = self.flow if self.flow_display else self.volume
            self.data_received("{:.2f}".format(value))
            self.sleep(1.0)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.reset_count = self.counter[self.gpio].pulse_count
        self.last_count = self.reset_count

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset Volume")
    def reset_button(self):
        self.reset()

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.volume_units
        if self.flow_display:
            unit += self.time_units
        return unit

################################################################################
@cbpi.sensor
class SimulatedFlowSensor(SensorActive):

    a_flow_actor_prop = Property.Actor("Actor", description="The actor this sensor responds to")
    a_flow_rate_prop = Property.Number("Flow Rate", configurable=True, default_value=2)
    b_display_prop = Property.Select("Display", options=["Volume", "Flow Rate"])
    c_volume_units_prop = Property.Text("Volume Units", configurable=True, default_value="L", description="Can by anything")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/s","/m"])

    #-------------------------------------------------------------------------------
    def init(self):
        try:
            self.flow_rate = float(self.a_flow_rate_prop)
            self.flow_actor = int(self.a_flow_actor_prop)
        except:
            self.flow_rate = 0.0
            self.flow_actor = None

        self.flow_display = self.b_display_prop == "Flow Rate"
        self.time_units = str(self.d_time_units_prop)
        self.period_adjust = 1.0/60.0 if (self.d_time_units_prop == "/m") else 1.0

        self.volume_units = "Units" if self.c_volume_units_prop == "" else self.c_volume_units_prop

        self.last_time = time.time()
        self.volume = 0.0
        self.flow = 0.0

        self.flow_device = None

        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def update_values(self):
        if self.flow_device:
            now = time.time()
            if self.flow_device and int(self.flow_device.state):
                self.flow = self.flow_rate * (float(self.flow_device.power) / 100.0)
                self.volume += self.flow * (now - self.last_time) * self.period_adjust
            else:
                self.flow = 0.0
            self.last_time = now

    #-------------------------------------------------------------------------------
    def execute(self):
        # at startup, wait for actors to initialze
        while cbpi.cache.get("actors") is None:
            self.sleep(5)

        self.flow_device = cbpi.cache.get("actors").get(self.flow_actor, None)
        while self.is_running():
            # an active brew step may have already updated values
            if time.time() - self.last_time > 1.0:
                self.update_values()
            value = self.flow if self.flow_display else self.volume
            self.data_received("{:.2f}".format(value))
            self.sleep(1.0)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.volume = 0.0

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset volume to zero")
    def reset_button(self):
        self.reset()

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.volume_units
        if self.flow_display:
            unit += self.time_units
        return unit

################################################################################
@cbpi.step
class FlowSensorStep(StepBase):
    a_sensor_prop = StepProperty.Sensor("Flow Sensor", description="Sensor that contols this step")
    b_actor1_prop = StepProperty.Actor("Actor 1", description="Actor to turn on for the duration of this step")
    c_actor2_prop = StepProperty.Actor("Actor 2", description="Actor to turn on for the duration of this step")
    d_volume_prop = Property.Number("Target Volume", configurable=True, description="Leave blank to continue until flow stops")
    e_resetStart_prop = Property.Select("Reset meter at start?", options=["Yes","No"])
    f_resetEnd_prop = Property.Select("Reset meter at end?", options=["Yes","No"])
    g_threshold_prop = Property.Number("Flow threshold", configurable=True, default_value=0.01, description="Value at which flow is considered stopped")

    #-------------------------------------------------------------------------------
    def init(self):
        self.sensor = cbpi.cache.get("sensors")[int(self.a_sensor_prop)].instance
        self.actors = [self.b_actor1_prop, self.c_actor2_prop]
        try: self.target_volume = float(self.d_volume_prop)
        except: self.target_volume = 0.0
        self.reset_start = self.e_resetStart_prop == "Yes"
        self.reset_end = self.f_resetEnd_prop == "Yes"
        try: self.threshold = float(self.g_threshold_prop)
        except: self.threshold = 0.01
        self.flowing = False

        if self.reset_start:
            self.sensor.reset()
        self.actors_on()

    #-------------------------------------------------------------------------------
    def reset(self):
        self.actors_off()

    #-------------------------------------------------------------------------------
    def finish(self):
        self.actors_off()
        self.notify("{} complete".format(self.name), "Total Volume: {:.2f} {}".format(self.sensor.volume, self.sensor.volume_units), timeout=None)
        if self.reset_end:
            self.sensor.reset()

    #-------------------------------------------------------------------------------
    def execute(self):
        self.sensor.update_values()
        if (not self.flowing) and (self.sensor.flow >= self.threshold):
            # flow has started
            self.flowing = True
        elif (self.flowing) and (self.sensor.flow <= self.threshold):
            # flow has stopped
            self.next()
        elif self.target_volume and (self.sensor.volume >= self.target_volume):
            # target volume met
            self.next()

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
    actor_prop = StepProperty.Actor("Actor")
    sensor_prop = StepProperty.Sensor("Sensor")
    timer_prop = Property.Number("Timer", configurable=True)
    threshold_prop = Property.Number("Flow threshold", configurable=True, default_value=0.01)

    #-------------------------------------------------------------------------------
    def init(self):
        self.actor = int(self.actor_prop)
        self.sensor = cbpi.cache.get("sensors")[int(self.a_sensor_prop)].instance
        self.threshold = float(self.threshold_prop)
        self.flowing = False

        self.sensor.reset()
        if self.is_timer_finished() is None:
            self.start_timer(int(self.timer_prop) * 60)
        self.actor_on(self.actor)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.actor_off(self.actor)

    #-------------------------------------------------------------------------------
    def finish(self):
        self.actor_off(self.actor)
        self.notify("Flow Sensor Calibration Complete", "Pulse Count: {}".format(self.sensor.pulse_count), timeout=None)

    #-------------------------------------------------------------------------------
    def execute(self):
        if (not self.flowing) and (self.sensor.flow >= self.threshold):
            self.flowing = True
        elif (self.flowing) and (self.sensor.flow <= self.threshold):
            self.next()
        elif self.is_timer_finished() == True:
            self.next()

################################################################################
@cbpi.step
class FlowSensorReset(StepBase):
    sensor_prop = StepProperty.Sensor("Sensor")

    #-------------------------------------------------------------------------------
    def init(self):
        self.sensor = cbpi.cache.get("sensors")[int(self.sensor_prop)].instance
        self.sensor.reset()
        self.next()
