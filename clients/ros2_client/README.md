# Mock Co-Simulator for GeoScenarioServer

Lock-step co-simulator node that drives GeoScenarioServer simulation forward by publishing delta_time values and controlling external vehicle motion.

## Lock-Step Flow

1. Server publishes current state on `/gs/tick`
2. Mock co-simulator receives state and updates external vehicles
3. Mock co-simulator publishes next step on `/gs/tick_from_client`
4. Server advances simulation by delta_time
5. Repeat until max_simulation_time reached

## Usage

```bash
ros2 run geoscenario_client mock_co_simulator
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_delta_time` | 0.025 | Simulation timestep in seconds (40 Hz) |
| `real_time_factor` | 1.0 | Speed multiplier (0=max speed, 1=real-time, >1=slower) |
| `max_simulation_time` | 30.0 | Auto-shutdown timeout in seconds (-1 for no limit) |

## Example

```bash
ros2 run geoscenario_client mock_co_simulator --ros-args \
  -p target_delta_time:=0.05 \
  -p real_time_factor:=0.5 \
  -p max_simulation_time:=60.0
```
