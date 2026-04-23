# Accel_Tool

C++ tool for working with MicroStrain accelerometers using the MSCL library. Built with C++, CMake, MSVC, and Ninja. MSCL is included as a local SDK under `third_party/mscl`.

## Project Structure
```bash
AccelTool
|-- CMakeLists.txt
|-- README.md
|-- src/          # application source code
|   `-- main.cpp
|-- include/      # project headers
|-- config/       # configuration files
|-- third_party/  # third-party libraries
|   `-- mscl/
|       |-- include/
|       |   |-- mscl/
|       |   |-- boost/
|       |   `-- openssl/
|       |-- lib/
|       |   `-- MSCL.lib
|       `-- bin/
|           `-- MSCL.dll
`-- build/        # build output (generated)
```

## Structure
```bash
MSCL Acquisition Thread
        |
        v
   Raw Sample Buffer
      |         |
      v         v
   Writer    Processing
                 |
                 v
         Processed Sample Buffer
             |             |
             v             v
            UI         Alarm/Monitor
```

## Requirements
Visual Studio Build Tools (MSVC)  
CMake >= 3.20  
Ninja

## Build
Open **x64 Native Tools Command Prompt for VS**
```bash
cd AccelTool
```

### Debug

```bash
cmake --preset debug
cmake --build --preset build-debug
```

### Release

```bash
cmake --preset release
cmake --build --preset build-release
```

## Run
```bash
build_debug\AccelTool.exe
build_release\AccelTool.exe
```

## Clean Build
If build configuration changes:
```bash
cd AccelTool
rmdir /s /q build
```

## Usage
```bash
AccelTool interactive mode
  I = Set to Idle + initialize device
  S = Start receiving data
  T = Stop receiving data
  Q = Quit
```

## CSV Output Fields

The application writes two CSV files:

- `accel_data.csv`: one processed accepted sample per row.
- `display_data.csv`: one display aggregation bucket per row.

### `accel_data.csv`

Header:

```csv
sample_index,node_address,device_tick,tick_gap_detected,tick_gap_count,device_timestamp_unix_ns,timestamp_gap_ns,timestamp_gap_detected,x,y,z,magnitude_xy,magnitude_xyz,norm_Lat_G,applied_spec,exceeds_spec
```

#### `sample_index`
Monotonic sample number assigned by this application after a sweep is accepted as a valid sample.

#### `node_address`
Wireless node address reported by the device.

#### `device_tick`
Sweep tick reported by MSCL from the wireless sync sampling packet.  
This is the primary sequence indicator used for data loss detection.

#### `tick_gap_detected`
Primary data loss flag.  
`1` means the current `device_tick` is not the expected next tick after the previous accepted sample.

#### `tick_gap_count`
Estimated number of missing ticks between the previous accepted sample and the current sample.  
`0` means no tick gap was detected.

#### `device_timestamp_unix_ns`
Full device timestamp in Unix epoch nanoseconds.  
This timestamp comes from the MSCL `DataSweep` device timestamp.

#### `timestamp_gap_ns`
Observed difference, in nanoseconds, between this sample's device timestamp and the previous accepted sample's device timestamp.

#### `timestamp_gap_detected`
Timestamp interval anomaly flag.  
`1` means `timestamp_gap_ns` falls outside the expected sample interval derived from `sampleRateHz`, using `timestampGapTolerancePercent` from the `.ini` file.

This is a secondary timing diagnostic. The primary loss indicator is still `tick_gap_detected`.

#### `x`
Acceleration value for the X axis.

#### `y`
Acceleration value for the Y axis.

#### `z`
Acceleration value for the Z axis.

#### `magnitude_xy`
Vector magnitude computed from X and Y:

```text
sqrt(x*x + y*y)
```

#### `magnitude_xyz`
Vector magnitude computed from X, Y, and Z:

```text
sqrt(x*x + y*y + z*z)
```

#### `norm_Lat_G`
Normalized lateral acceleration value computed as:

```text
magnitude_xy / z
```

If `z` is `0`, this field is written as `0`.

#### `applied_spec`
Configured spec threshold from the `.ini` file.

#### `exceeds_spec`
Spec exceedance flag.  
`1` means the selected magnitude exceeded `applied_spec`.

The selected magnitude depends on `axisMode`:

- `XY`: compares `magnitude_xy` against `applied_spec`.
- `XYZ`: compares `magnitude_xyz` against `applied_spec`.

### `display_data.csv`

Header:

```csv
bucket_index,start_sample_index,end_sample_index,start_device_timestamp_unix_ns,end_device_timestamp_unix_ns,sample_count,peak_x,peak_y,peak_z,max_magnitude_xy,max_magnitude_xyz,max_norm_Lat_G
```

#### `bucket_index`
Monotonic display bucket number assigned by the application.

#### `start_sample_index`
First `sample_index` included in this display bucket.

#### `end_sample_index`
Last `sample_index` included in this display bucket.

#### `start_device_timestamp_unix_ns`
Device timestamp, in Unix epoch nanoseconds, for the first sample in the bucket.

#### `end_device_timestamp_unix_ns`
Device timestamp, in Unix epoch nanoseconds, for the last sample in the bucket.

#### `sample_count`
Number of accepted processed samples included in the bucket.

This is normally controlled by `displayAggregationSamples` in the `.ini` file.  
The final flushed bucket may contain fewer samples.

#### `peak_x`
Signed X value with the largest absolute value in the bucket.

#### `peak_y`
Signed Y value with the largest absolute value in the bucket.

#### `peak_z`
Signed Z value with the largest absolute value in the bucket.

#### `max_magnitude_xy`
Maximum `magnitude_xy` value in the bucket.

#### `max_magnitude_xyz`
Maximum `magnitude_xyz` value in the bucket.

#### `max_norm_Lat_G`
Maximum `norm_Lat_G` value in the bucket.

## Notes On Timing And Loss Detection

### Device timestamp
`device_timestamp_unix_ns` comes from the device timestamp parsed by MSCL.  
The current CSV output does not include a host-side timestamp column.

### Primary loss indicator
`tick_gap_detected` and `tick_gap_count` are the main fields for identifying possible data loss.

### Secondary timing diagnostic
`timestamp_gap_ns` and `timestamp_gap_detected` provide a per-sample timing consistency check.  
The expected interval is derived from the configured `sampleRateHz`, and the allowed tolerance is controlled by `timestampGapTolerancePercent`.

### Accepted samples only
CSV rows are written only for sweeps that passed channel validation and were accepted as valid samples by the application.
