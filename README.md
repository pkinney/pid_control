# PIDControl

![Build Status](https://github.com/pkinney/pid_control/actions/workflows/ci.yaml/badge.svg)
[![Hex.pm](https://img.shields.io/hexpm/v/pid_control.svg)](https://hex.pm/packages/pid_control)
[![Documentation](https://img.shields.io/badge/documentation-gray)](https://hexdocs.pm/pid_control)

A discrete implementation of a [PID controller](https://en.wikipedia.org/wiki/PID_controller) for building
closed-loop control into embedded systems.

![PID](/images/pid.png)

## Installation

The package can be installed by adding `pid_control` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:pid_control, "~> 0.1.0"}
  ]
end
```

## Usage

The `PIDControl.new/1` function returns an initialized `PIDControl` struct with the given configuration:

```elixir
iex> pid = PIDControl.new(kp: 0.5, kd: 0.2, ki: 0.03)
%PIDControl{...}
```

Calling `PIDControl.step/3` performs one discrete cycle of the PID loop for the given `set_point` and `measurement`
values.  The new PIDControl state is returned and the output can be accessed via the `output` key in the struct.

```elixir
iex> pid = PIDControl.step(pid, 0.3, 0.312)
%PIDControl{
  output: -0.11448221
  #...
}
```

The `step` function can be called in each successive input-output cycle of the underling sensor/actuator.

## Example 

```elixir
defmodule Controller do
  use GenServer
  # ...

  def init(_) do
    Sensor.subcribe()
    pid = PIDControl.new(kp: 0.5, kd: 0.2, ki: 0.03)
    {:ok, pid}
  end

  def handle_info({:sensor, measurment}, pid) do
    pid = PIDControl.step(pid, @set_point, measurment)
    Actuator.command(pid.output)
    {:noreply, pid}
  end
end
```

## Config Options

  * **`kp`, `ki`, `kp`** - Parameters for each term in the PID.  Any left blank will be set to `0`
  * **`tau`** - Low-pass filter parameter for the calculated `d` term.  A value of `1` will bypass the filtering.
    A value between `0` and `1` will filter out high-frequency noise in the derivative term.
  * **`t`** - Time factor for calculating the `i` and `d` term of the PID.  A value of `1` will ignore
    any time parameters and treat each `step` as a single time unit.  A value of `0.1`, for example,
    will treat each `step` as a tenth of a time unit.  If `use_system_t` is set to `true`, this value
    is ignored in favor of the system time (in seconds) between calls to `step`.
  * **`output_min` and `output_max`** - Minimum and maximum values that will be output from the PID. Any other
    values outside of this range will be clamped. These same values are used for anti-windup of the `i` term.
    Default is `-1` and `1`, respectively.
  * **`use_system_t`** - When set to `true`, the time used to calculate `d` and `i` terms of the PID will be
    derived from the measured time since the last call to `step` (using `System.monotonic_time/0`). When set
    to false, the configured value of `t` will be used. Defaults to `false`.
  * **`zero_d_on_set_point_change`** - When the set point changes rapidly, the d-term will suddenly spike, causing
    a sudden change in the PID output. Setting `tau` to a lower value will lessen this effect, but setting 
    `zero_d_on_set_point_change` will force the d-term to zero for the step when the set_point changes.  This is 
    useful for situations when the set point changes suddenly and occasionally, as opposed to situations where 
    the set_point is gradually changes (as when following a profile curve).  Defaults to `false`.
  * **`telemetry`** - When set to `true`, the PID will automatically emit its own telemetry after each `step`.
    Defaults to `false`.
  * **`telemetry_prefix`** - If `telemetry` is set to `true`, this value defines the name that the event is published
    under.  Defaults to `[:pid_control]`

## Telemetry

If `telemetry` is set to `true`, telemetry for the PID state will be emitted each time the `step` function is called.
By default, the event name will be `[pid_control]` and will contain the following measurements.

* `set_point` - Current set point
* `measurement` - Most recent measurement
* `error` - Current error that is being fed to the PID
* `p` - Proportional component of the output
* `i` - Integral component of the output
* `d` - Derivative component of the output
* `t` - The time value used for the step function.  This will always be the configured value `t` unless `use_system_t` is set to `true`.
* `output` - Most recent output (which will be the sum of `p`, `i`, and `d` values)

This is really useful for manual tuning when combined with something like `PheonixLiveDashboard`.
