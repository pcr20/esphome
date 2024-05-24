# ESPHome [![Discord Chat](https://img.shields.io/discord/429907082951524364.svg)](https://discord.gg/KhAMKrd) [![GitHub release](https://img.shields.io/github/release/esphome/esphome.svg)](https://GitHub.com/esphome/esphome/releases/)

[![ESPHome Logo](https://esphome.io/_images/logo-text.png)](https://esphome.io/)

**Documentation:** https://esphome.io/

For issues, please go to [the issue tracker](https://github.com/esphome/issues/issues).

For feature requests, please see [feature requests](https://github.com/esphome/feature-requests/issues).

This branch contains modified versions of modbus and modbus_controller which includes an option to disable sending:
~~~
modbus_controller:
  - id: myid
    address: 0x0001
    modbus_id: modbus1
    update_interval: 10s
    setup_priority: -10
    command_throttle: 0ms
    disable_send: True
~~~
The components can be used by using the external_components feature in your yaml:
~~~
external_components:
  - source:
      type: git
      url: https://github.com/pcr20/esphome
      ref: dev
    components: [ modbus, modbus_controller ]
    refresh: 2minutes
~~~
The data is retrieved by creating a sensor - use a lambda to access the data:
~~~
sensor:
  - platform: modbus_controller
    modbus_controller_id: orno_we_504
    name: alldata
    id: alldata_id
    # 0x1 : modbus device address
    # 0x3 : modbus function code (read holding)
    # 0x00 : high byte of modbus register address
    # 0x00: low byte of modbus register address
    # 0x00: high byte of total number of registers requested
    # 0x11: low byte of total number of registers requested
    custom_command: [ 0x1, 0x3, 0x00, 0x00,0x00, 0x11]
    lambda: |-
      ESP_LOGD("Modbus Sensor Lambda","s: %d %02d %02d %02d %02d",data.size(),data[0],data[1],data[2],data[3],data[4]);
      return 0;
~~~
