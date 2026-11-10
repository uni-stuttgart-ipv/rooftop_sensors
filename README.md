# Wireless Sensor Network for Rooftop Solar Modules
A wireless sensor network for monitoring rooftop solar panels in real time.  
The system collects electrical and environmental parameters from distributed sensor nodes installed on solar modules and stores the data in InfluxDB for monitoring, analysis, and visualization.
> For access to our solar spectrum data send an email to sekretariat@ipv.uni-stuttgart.de.

## Setup
### WiFi
Set the WiFi SSID and password in the environment file.

### InfluxDB (required)
Set the environment variables
- `INFLUX_URL` to the influx db url
- `INFLUX_TOKEN` to the influx db access token
- `INFLUX_ORG` to the influx organisation detail
- `INFLUX_BUCKET` to the bucket to which data needs to be pushed

### Running the program
After flashing the code initially panel details need to be entered via UART using the command:
- `set_panel string panel_sn location boxType`

## Database Structure

The system stores sensor data in **InfluxDB** under multiple measurements.

### 1. Measurement: `panel_parameters`

Stores electrical and panel-level measurements.

#### Tags

- `mcu`: Microcontroller MAC address sending the measurements  
- `panel_sn`: Serial number of the physical panel on which the MCU is installed  
- `string`: Panel manufacturer and installation reference (based on rooftop map)  
- `location`: Panel location in format - panelgroup_rowNumber_columnNumber (Example panel groups: A, B,C)

#### Fields (Measured Parameters)

- `voltage`  
- `current`  
- `power`  
- `temperature`: Surface temperature of the panel  


### 2. Measurement: `temperature_humidity`

Stores environmental sensor data.

#### Tag

- `mcu`

#### Fields

- `ambient_temperature`: Surrounding air temperature  
- `humidity`: Ambient humidity level  

### 3. Measurement: `irradiance`

Stores solar irradiance data.

#### Tag

- `mcu`   

#### Field

- `irradiance`: Solar irradiance incident on the panel  
Wireless Sensor Network for Rooftop Solar modules
