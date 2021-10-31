"""Module to simulate interaction with an HX711 ADC"""

import time
import random
import math
import threading

class HX711:
    """Class to simulate interaction with an HX711 ADC"""
    def __init__(self, dout, pd_sck, gain=128):
        self.pd_sck = pd_sck

        self.dout = dout

        # Last time we've been read.
        self.last_read_time = time.time()
        self.sample_rate_hz = 80.0
        self.reset_timestamp = time.time()
        self.sample_count = 0
        self.simulate_tare = False

        # Mutex for reading from the HX711, in case multiple threads in client
        # software try to access get values from the class at the same time.
        self.read_lock = threading.Lock()

        self.gain = 0
        # The value returned by the hx711 that corresponds to your reference unit
        # AFTER dividing by the SCALE.
        self.reference_unit = 1

        self.offset = 1
        self.last_val = int(0)

        self.debug_printing = False

        self.byte_format = 'MSB'
        self.bit_format = 'MSB'

        self.set_gain(gain)

        # Think about whether this is necessary.
        time.sleep(1)

    def convert_to_twos_complement_24_bit(self, input_value):
        # HX711 has saturating logic.
        if input_value >= 0x7fffff:
            return 0x7fffff

        # If it's a positive value, just return it, masked with our max value.
        if input_value >= 0:
            return input_value & 0x7fffff

        if input_value < 0:
            # HX711 has saturating logic.
            if input_value < -0x800000:
                input_value = -0x800000

            diff = input_value + 0x800000

            return 0x800000 + diff

    def convert_from_twos_complement_24_bit(self, input_value):
        return -(input_value & 0x800000) + (input_value & 0x7fffff)

    def is_ready(self):
        # Calculate how long we should be waiting between samples, given the
        # sample rate.
        sample_delay_seconds = 1.0 / self.sample_rate_hz

        return time.time() >= self.last_read_time + sample_delay_seconds

    def set_gain(self, gain):
        if gain == 128:
            self.gain = 1
        elif gain == 64:
            self.gain = 3
        elif gain == 32:
            self.gain = 2

        # Read out a set of raw bytes and throw it away.
        self.read_raw_bytes()

    def get_gain(self):
        if self.gain == 1:
            return 128
        if self.gain == 3:
            return 64
        if self.gain == 2:
            return 32

        # Shouldn't get here.
        return 0

    def read_raw_bytes(self):
        # Wait for and get the Read Lock, incase another thread is already
        # driving the virtual HX711 serial interface.
        with self.read_lock:

            # Wait until HX711 is ready for us to read a sample.
            while not self.is_ready():
                pass

            self.last_read_time = time.time()

            # Generate a 24bit 2s complement sample for the virtual HX711.
            raw_sample = self.convert_to_twos_complement_24_bit(
                self.generate_fake_sample())

            # Read three bytes of data from the HX711.
            first_byte = (raw_sample >> 16) & 0xFF
            second_byte = (raw_sample >> 8) & 0xFF
            third_byte = raw_sample & 0xFF

        # Depending on how we're configured, return an orderd list of raw byte
        # values.
        if self.byte_format == 'LSB':
            return [third_byte, second_byte, first_byte]
        return [first_byte, second_byte, third_byte]

    def read_long(self):
        # Get a sample from the HX711 in the form of raw bytes.
        data_bytes = self.read_raw_bytes()

        if self.debug_printing:
            print(data_bytes,)

        # Join the raw bytes into a single 24bit 2s complement value.
        twos_complement_value = ((data_bytes[0] << 16) |
                               (data_bytes[1] << 8) |
                               data_bytes[2])

        if self.debug_printing:
            print("Twos: 0x%06x" % twos_complement_value)

        # Convert from 24bit twos-complement to a signed value.
        signed_int_value = self.convert_from_twos_complement_24_bit(
            twos_complement_value)

        # Record the latest sample value we've read.
        self.last_val = signed_int_value

        # Return the sample value we've read from the HX711.
        return int(signed_int_value)

    def read_average(self, times=3):
        # Make sure we've been asked to take a rational amount of samples.
        if times <= 0:
            print("HX711().read_average(): times must >= 1!!  Assuming value of 1.")
            times = 1

        # If we're only average across one value, just read it and return it.
        if times == 1:
            return self.read_long()

        # If we're averaging across a low amount of values, just take an
        # arithmetic mean.
        if times < 5:
            values = int(0)
            for _ in range(times):
                values += self.read_long()

            return values / times

        # If we're taking a lot of samples, we'll collect them in a list, remove
        # the outliers, then take the mean of the remaining set.
        value_list = []

        for _ in range(times):
            value_list += [self.read_long()]

        value_list.sort()

        # We'll be trimming 20% of outlier samples from top and bottom of collected set.
        trim_amount = int(len(value_list) * 0.2)

        # Trim the edge case values.
        value_list = value_list[trim_amount:-trim_amount]

        # Return the mean of remaining samples.
        return sum(value_list) / len(value_list)

    def get_value(self, times=3):
        return self.read_average(times) - self.offset

    def get_weight(self, times=3):
        value = self.get_value(times)
        value = value / self.reference_unit
        return value

    def tare(self, times=15):
        # If we aren't simulating Taring because it takes too long, just skip it.
        if not self.simulate_tare:
            return 0

        # Backup REFERENCE_UNIT value
        reference_unit = self.reference_unit
        self.set_reference_unit(1)

        value = self.read_average(times)

        if self.debug_printing:
            print("Tare value:", value)

        self.set_offset(value)

        # Restore the reference unit, now that we've got our offset.
        self.set_reference_unit(reference_unit)

        return value

    def set_reading_format(self, byte_format="LSB", bit_format="MSB"):

        if byte_format == "LSB":
            self.byte_format = byte_format
        elif byte_format == "MSB":
            self.byte_format = byte_format
        else:
            print(f"Unrecognised byte_format: \"{byte_format}\"")

        if bit_format == "LSB":
            self.bit_format = bit_format
        elif bit_format == "MSB":
            self.bit_format = bit_format
        else:
            print(f"Unrecognised bit_format: \"{bit_format}\"")

    def set_offset(self, offset):
        self.offset = offset

    def get_offset(self):
        return self.offset

    def set_reference_unit(self, reference_unit):
        # Make sure we aren't asked to use an invalid reference unit.
        if reference_unit == 0:
            print("HX711().set_reference_unit(): Can't use 0 as a reference unit!!")
            return

        self.reference_unit = reference_unit

    def power_down(self):
        # Wait for and get the Read Lock, incase another thread is already
        # driving the HX711 serial interface.
        with self.read_lock:

            # Wait 100us for the virtual HX711 to power down.
            time.sleep(0.0001)

    def power_up(self):
        # Wait for and get the Read Lock, incase another thread is already
        # driving the HX711 serial interface.
        with self.read_lock:

            # Wait 100 us for the virtual HX711 to power back up.
            time.sleep(0.0001)

        # HX711 will now be defaulted to Channel A with gain of 128.  If this
        # isn't what client software has requested from us, take a sample and
        # throw it away, so that next sample from the HX711 will be from the
        # correct channel/gain.
        if self.get_gain() != 128:
            self.read_raw_bytes()

    def reset(self):
        # self.power_down()
        # self.power_up()

        # Mark time when we were reset.  We'll use this for sample generation.
        self.reset_timestamp = time.time()

    def generate_fake_sample(self):
        sample_timestamp = time.time() - self.reset_timestamp

        noise_scale = 1.0
        noise_value = random.randrange(-(noise_scale * 1000),
                                      (noise_scale * 1000)) / 1000.0
        sample = math.sin(math.radians(sample_timestamp * 20)) * 72.0

        self.sample_count += 1

        if sample < 0.0:
            sample = -sample

        sample += noise_value

        big_error_sample_frequency = 142
        big_error_samples = [0.0, 40.0, 70.0, 150.0, 280.0, 580.0]

        if random.randrange(0, big_error_sample_frequency) == 0:
            sample = random.sample(big_error_samples, 1)[0]
            print(f"Sample {self.sample_count}: Injecting {sample} as a random bad sample.")

        sample *= 1000

        sample *= self.reference_unit

        return int(sample)
