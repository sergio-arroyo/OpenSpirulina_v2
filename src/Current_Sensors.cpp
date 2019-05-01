/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Current_Sensors class used to control invasive & non-invasive current sensors attached to the system
 * 
 * Based on:
 *    DFRobot code: https://wiki.dfrobot.com/Gravity_Analog_AC_Current_Sensor__SKU_SEN0211_
 * 
 *    Naylamp Mechatronics code: https://naylampmechatronics.com/blog/48_tutorial-sensor-de-corriente-acs712.html
 * 
 */

#include "Current_Sensors.h"


Current_Sensors::Current_Sensors() {
    v_ref = read_V_ref();
    n_sensors = 0;
}

bool Current_Sensors::add_sensor(uint8_t pin, Current_Model_t model, uint16_t var) {
    if (n_sensors >= CURR_MAX_NUM_SENSORS) return false;

    sensors[n_sensors].model = model;
    sensors[n_sensors].pin = pin;
    sensors[n_sensors].var = var;
    n_sensors++;

    return true;
}

const float Current_Sensors::get_current_value(uint8_t n_sensor) {
    if (n_sensor >= n_sensors) return 0;

    switch (sensors[n_sensor].model) {
        case ACS712:
            return get_current_ACS712(n_sensor);
            break;
        case SCT013:
            return get_current_SCT013(n_sensor);
            break;
    }

    return -127;
}

void Current_Sensors::capture_all_sensors() {
    for (uint8_t i=0; i<n_sensors; i++) {
        arr_current[i] = get_current_value(i);
    }
}

/* Obtain current value on specific invasive sensor ACS712 */
const float Current_Sensors::get_current_ACS712(uint8_t n_sensor) {
    float sens_val = 0;

    for (uint8_t i=0; i<CURR_SENS_N_SAMPLES; i++) {
        sens_val += (float) analogRead(sensors[n_sensor].pin);
        delay(CURR_SENS_MS_INTERV);
    }
    sens_val = (sens_val / CURR_SENS_N_SAMPLES) * (5.0 / 1023.0);

    float ac_offset = v_ref / 2000.0;
    float sensitivity = (sensors[n_sensor].var / 1000.0) * (v_ref / 5000.0);

    return (sens_val - ac_offset) / sensitivity;
}

/* Obtain current value on specific non-invasive sensor SCT013 */
const float Current_Sensors::get_current_SCT013(uint8_t n_sensor) {
    unsigned int peakVoltage = 0;

    for (uint8_t i=0; i<CURR_SENS_N_SAMPLES; i++) {
        peakVoltage += analogRead(sensors[n_sensor].pin);  // Read peak voltage
        delay(CURR_SENS_MS_INTERV);
    }

    peakVoltage /= CURR_SENS_N_SAMPLES;

    float v_rms = peakVoltage * 0.707;                     // Change the peak voltage to the Virtual Value of voltage
    v_rms = (v_rms * v_ref / 1024.0) / 2.0;                // The circuit is amplified by 2 times, so it is divided by 2

    return (v_rms * sensors[n_sensor].var) / 1000;
}

const uint8_t Current_Sensors::get_n_sensors() {
    return n_sensors;
}

const uint16_t Current_Sensors::get_volt_ref() {
    return v_ref;
}

void Current_Sensors::set_volt_ref(uint16_t _v_ref) {
    v_ref = _v_ref;
}

/* Read reference voltage */
uint16_t Current_Sensors::read_V_ref() {
    uint16_t result;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB &= ~_BV(MUX5);                                  // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#endif
#if defined(__AVR__)
    delay(2);                                              // Wait for Vref to settle
    ADCSRA |= _BV(ADSC);                                   // Convert
    while (bit_is_set(ADCSRA, ADSC));
    result = ADCL;
    result |= ADCH << 8;
    result = 1126400L / result;                            // 1100mV * 1024 ADC steps http://openenergymonitor.org/emon/node/1186
    return result;
#elif defined(__arm__)
    return (3300);                                         // Arduino Due
#else
    return (3300);                                         // Guess that other un-supported architectures will be running a 3.3V!
#endif
}