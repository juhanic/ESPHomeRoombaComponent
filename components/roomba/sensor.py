import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor, uart
from esphome.const import CONF_ID, UNIT_EMPTY, ICON_EMPTY, UNIT_MILLIAMP, UNIT_MILLIMETER, ICON_RULER, UNIT_VOLT, UNIT_WATT_HOURS, UNIT_CELSIUS, UNIT_PERCENT, CONF_PIN
roomba_component_ns = cg.esphome_ns.namespace("roomba")

RoombaComponent = roomba_component_ns.class_("RoombaComponent", cg.PollingComponent, uart.UARTDevice)

CONF_DISTANCE_SENSOR = "distance_sensor";
CONF_VOLTAGE_SENSOR = "voltage_sensor";
CONF_CURRENT_SENSOR = "current_sensor";
CONF_CHARGE_SENSOR = "charge_sensor";
CONF_CAPACITY_SENSOR = "capacity_sensor";
CONF_BATTERY_SENSOR = "battery_percentage_sensor";
CONF_TEMPERATURE_SENSOR = "temperature_sensor";
CONF_ACTIVITY_SENSOR = "activity_sensor";


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RoombaComponent),
    cv.Required(CONF_PIN): pins.internal_gpio_input_pullup_pin_schema,
    cv.Optional(CONF_DISTANCE_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_MILLIMETER, icon=ICON_RULER, accuracy_decimals=1),
    cv.Optional(CONF_VOLTAGE_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, icon=ICON_EMPTY, accuracy_decimals=1),
    cv.Optional(CONF_CURRENT_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_MILLIAMP, icon=ICON_EMPTY, accuracy_decimals=1),
    cv.Optional(CONF_CHARGE_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_WATT_HOURS, icon=ICON_EMPTY, accuracy_decimals=1),
    cv.Optional(CONF_CAPACITY_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_WATT_HOURS, icon=ICON_EMPTY, accuracy_decimals=1),
    cv.Optional(CONF_BATTERY_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, icon=ICON_EMPTY, accuracy_decimals=1),
    cv.Optional(CONF_TEMPERATURE_SENSOR):
      sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_EMPTY, accuracy_decimals=1),
#    cv.Optional(CONF_ACTIVITY_SENSOR): text_sensor.text_sensor_schema(),
}).extend(cv.polling_component_schema("60s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if dist_config := config.get(CONF_DISTANCE_SENSOR):
        sens = await sensor.new_sensor(dist_config)
        cg.add(var.set_distance_sensor(sens))
    if volt_config := config.get(CONF_VOLTAGE_SENSOR):
        sens = await sensor.new_sensor(volt_config)
        cg.add(var.set_voltage_sensor(sens))
    if current_config := config.get(CONF_CURRENT_SENSOR):
        sens = await sensor.new_sensor(current_config)
        cg.add(var.set_current_sensor(sens))
    if charge_config := config.get(CONF_CHARGE_SENSOR):
        sens = await sensor.new_sensor(charge_config)
        cg.add(var.set_charge_sensor(sens))
    if capacity_config := config.get(CONF_CAPACITY_SENSOR):
        sens = await sensor.new_sensor(capacity_config)
        cg.add(var.set_capacity_sensor(sens))
    if battery_config := config.get(CONF_BATTERY_SENSOR):
        sens = await sensor.new_sensor(battery_config)
        cg.add(var.set_battery_sensor(sens))
    if temp_config := config.get(CONF_TEMPERATURE_SENSOR):
        sens = await sensor.new_sensor(temp_config)
        cg.add(var.set_temperature_sensor(sens))
#    if activity_config := config.get(CONF_ACTIVITY_SENSOR):
#        sens = await text_sensor.new_text_sensor(CONF_ACTIVITY_SENSOR)
#        cg.add(var.set_text_sensor(sens))
