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
    pass

################################################################################
@cbpi.sensor
class FlowSensor(SensorActive):
    a_gpio_prop = Property.Select("GPIO", options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27])
    b_display_prop = Property.Select("Display", options=["Volume", "Flow"])
    c_volume_units_prop = Property.Select("Volume Units", options=["L","gal","qt"], description="Some text")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/sec","/min"])
    e_calibration_units_prop = Property.Number("Calibration Units", configurable=True, default_value=1, description="Actual units transferred during calibration test")
    f_calibration_count_prop = Property.Number("Calibration Count", configurable=True, default_value=485, description="Reported count from calibration test")

    #-------------------------------------------------------------------------------
    def init(self):
        self.gpio = int(self.a_gpio_prop)
        self.flow_display = self.b_display_prop == "Flow"
        self.per_minute = self.d_time_units_prop == "/min"
        self.calibration = float(self.e_calibration_units_prop)/float(self.f_calibration_count_prop)
        if self.c_volume_units_prop not in ["L","gal","qt"]:
            self.volume_units = "Units"
        else:
            self.volume_units = self.c_volume_units_prop

        self.pulse_count = 0
        self.last_count = 0
        self.last_time = time.time()
        self.volume = 0.0
        self.flow = 0.0

        try:
            GPIO.setup(self.gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.gpio, GPIO.RISING, callback=self.pulseInput, bouncetime=20)
        except Exception as e:
            print e

        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def pulseInput(self, channel):
        self.pulse_count += 1

    #-------------------------------------------------------------------------------
    def datum(self):
        now = time.time()
        count = self.pulse_count
        count_delta = count - self.last_count
        self.volume = count * self.calibration
        self.flow = (count_delta * self.calibration)/(now - self.last_time)
        if self.per_minute:
            self.flow /= 60.0
        self.last_count = count
        self.last_time = now

    #-------------------------------------------------------------------------------
    def execute(self):
        while self.is_running():
            self.datum()
            if self.flow_display:
                value = self.flow
            else:
                value = self.volume
            self.data_received("{:.2f}".format(value))
            self.sleep(1)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.pulse_count = self.last_count = 0

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset volume to zero")
    def resetButton(self):
        self.reset()

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.c_volume_units_prop
        if self.b_display_prop == "Flow":
            unit += self.d_time_units_prop
        return unit

################################################################################
@cbpi.sensor
class SimulatedFlowSensor(SensorActive):

    a_flow_actor_prop = Property.Actor("Actor", description="The actor this sensor responds to")
    a_flow_rate_prop = Property.Number("Flow Rate", configurable=True, default_value=2)
    b_display_prop = Property.Select("Display", options=["Volume", "Flow"])
    c_volume_units_prop = Property.Select("Volume Units", options=["L","gal","qt"], description="Some text")
    d_time_units_prop = Property.Select("Flow Time Units", options=["/sec","/min"])

    #-------------------------------------------------------------------------------
    def init(self):
        try:
            self.flow_rate = float(self.a_flow_rate_prop)
            self.flow_actor = int(self.a_flow_actor_prop)
        except:
            self.flow_rate = 0.0
            self.flow_actor = None

        self.flow_display = self.b_display_prop == "Flow"
        self.per_minute = self.d_time_units_prop == "/min"
        if self.c_volume_units_prop not in ["L","gal","qt"]:
            self.volume_units = "Units"
        else:
            self.volume_units = self.c_volume_units_prop

        if self.per_minute:
            self.flow_period_adjust = 1.0/60.0
        else:
            self.flow_period_adjust = 1.0

        self.volume = 0.0
        self.flow = 0.0

        SensorActive.init(self)

    #-------------------------------------------------------------------------------
    def execute(self):
        # at startup, wait for actors to initialze
        while cbpi.cache.get("actors") is None:
            self.sleep(5)

        while self.is_running():
            flow_device = cbpi.cache.get("actors").get(self.flow_actor, None)

            if flow_device and int(flow_device.state):
                self.volume += self.flow_rate * (float(flow_device.power) / 100.0) * self.flow_period_adjust
                self.flow = self.flow_rate
            else:
                self.flow = 0.0

            if self.flow_display:
                value = self.flow
            else:
                value = self.volume
            self.data_received("{:.2f}".format(value))

            self.sleep(1)

    #-------------------------------------------------------------------------------
    def reset(self):
        self.volume = 0.0

    #-------------------------------------------------------------------------------
    @cbpi.action("Reset volume to zero")
    def resetButton(self):
        self.reset()

    #-------------------------------------------------------------------------------
    def get_unit(self):
        unit = self.c_volume_units_prop
        if self.b_display_prop == "Flow":
            unit += self.d_time_units_prop
        return unit

################################################################################
@cbpi.step
class FlowSensorStep(StepBase):
    a_sensor_prop = StepProperty.Sensor("Sensor")
    b_actor1_prop = StepProperty.Actor("Actor 1")
    c_actor2_prop = StepProperty.Actor("Actor 2")
    d_volume_prop = Property.Number("Volume (blank=until flow stops)", configurable=True)
    e_resetStart_prop = Property.Select("Reset meter at start?", options=["Yes","No"])
    f_resetEnd_prop = Property.Select("Reset meter at end?", options=["Yes","No"])
    g_threshold_prop = Property.Number("Flow threshold", configurable=True, default_value=0.01)

    #-------------------------------------------------------------------------------
    def init(self):
        self.sensor = cbpi.cache.get("sensors")[int(self.a_sensor_prop)].instance
        self.actors = [self.b_actor1_prop, self.c_actor2_prop]
        try:
            self.target_volume = float(self.d_volume_prop)
            self.until_empty = False
        except:
            self.target_volume = 0.0
            self.until_empty = True
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
        cbpi.notify("Flow Sensor Step Complete", "Total Volume: {:.2f} {}".format(self.sensor.volume, self.sensor.volume_units), timeout=None)
        if self.reset_end:
            self.sensor.reset()

    #-------------------------------------------------------------------------------
    def execute(self):
        if self.until_empty:
            if (not self.flowing) and (self.sensor.flow >= self.threshold):
                self.flowing = True
            elif (self.flowing) and (self.sensor.flow <= self.threshold):
                self.next()
        else:
            if self.sensor.volume >= self.target_volume:
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
        cbpi.notify("Flow Sensor Calibration Complete", "Pulse Count: {}".format(self.sensor.pulse_count), timeout=None)

    #-------------------------------------------------------------------------------
    def execute(self):
        if self.is_timer_finished() == True:
            self.next()
        elif (not self.flowing) and (self.sensor.flow >= self.threshold):
            self.flowing = True
        elif (self.flowing) and (self.sensor.flow <= self.threshold):
            self.next()

################################################################################
@cbpi.step
class FlowSensorReset(StepBase):
    sensor_prop = StepProperty.Sensor("Sensor")

    #-------------------------------------------------------------------------------
    def init(self):
        self.sensor = cbpi.cache.get("sensors")[int(self.a_sensor_prop)].instance
        self.sensor.reset()
        self.next()
