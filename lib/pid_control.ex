defmodule PIDControl do
  @moduledoc """
  A discrete implementation of a [PID controller](https://en.wikipedia.org/wiki/PID_controller) for building
  closed-loop control into embedded systems.

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

  """
  defstruct p: 0.0,
            i: 0.0,
            d: 0.0,
            t: 0.0,
            e0: nil,
            measurement_prev: 0.0,
            set_point_prev: 0.0,
            output: 0.0,
            time_prev: nil,
            config: %{}

  @default_config %{
    kp: 0.0,
    ki: 0.0,
    kd: 0.0,
    tau: 1.0,
    t: 1.0,
    output_min: -1.0,
    output_max: 1.0,
    use_system_t: false,
    zero_d_on_set_point_change: false,
    telemetry: false,
    telemetry_event_name: [:pid_control]
  }

  @type config() :: %{
          kp: float(),
          ki: float(),
          kd: float(),
          tau: float(),
          t: float(),
          output_min: float(),
          output_max: float(),
          use_system_t: boolean(),
          zero_d_on_set_point_change: boolean(),
          telemetry: boolean(),
          telemetry_event_name: list(atom())
        }
  @type t() :: %__MODULE__{
          p: float(),
          i: float(),
          d: float(),
          t: float(),
          e0: float() | nil,
          measurement_prev: float(),
          set_point_prev: float(),
          output: float(),
          time_prev: integer() | nil,
          config: config()
        }

  @doc """
  Creates a new PID controller.

  Config Options:

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
  """
  @spec new(keyword()) :: t()
  def new(config \\ []) do
    %__MODULE__{config: @default_config} |> apply_config(config)
  end

  defp apply_config(%__MODULE__{} = pid, config) do
    pid |> Map.put(:config, Map.merge(pid.config, Enum.into(config, %{})))
  end

  @doc """
  Performs one step in the control loop.  Given the set point and measurement, the PID state
  is advanced and the output of the PID is set to the `output` field of the `PIDControl` struct.
  """
  @spec step(t(), float(), float()) :: t()
  def step(pid, set_point, measurement) do
    time = System.monotonic_time()

    t =
      if pid.config.use_system_t do
        t_actual(pid, time)
      else
        pid.config.t
      end

    e0 = set_point - measurement

    # P term is simply linear with the error
    p = pid.config.kp * e0

    # D term is linear on the change in error, passed through a low-pass filter
    last_error =
      cond do
        pid.e0 == nil -> e0
        set_point != pid.set_point_prev and pid.config.zero_d_on_set_point_change -> e0
        true -> pid.e0
      end

    d_raw = pid.config.kd * (e0 - last_error) / max(t, 0.001)
    d = pid.d + pid.config.tau * (d_raw - pid.d)

    # I term is linear to the sum of all previous errors, with anti-windup
    i_raw = pid.i + pid.config.ki * e0 * t
    i = clamp(i_raw, pid.config.output_min, pid.config.output_max)

    output = clamp(p + d + i, pid.config.output_min, pid.config.output_max)

    %{
      pid
      | p: p,
        i: i,
        d: d,
        t: t,
        e0: e0,
        measurement_prev: measurement,
        set_point_prev: set_point,
        output: output,
        time_prev: time
    }
    |> emit_telemetry()
  end

  defp t_actual(pid, time) do
    System.convert_time_unit(
      time - (Map.get(pid, :time_prev) || time),
      :native,
      :microsecond
    ) / 1_000_000.0
  end

  defp emit_telemetry(%__MODULE__{config: %{telemetry: true}} = pid) do
    :telemetry.execute(
      pid.config.telemetry_event_name,
      %{
        set_point: pid.set_point_prev,
        measurement: pid.measurement_prev,
        error: pid.e0,
        output: pid.output,
        p: pid.p,
        i: pid.i,
        d: pid.d,
        t: pid.t
      }
    )

    pid
  end

  defp emit_telemetry(pid), do: pid

  defp clamp(n, a, b) when a > b, do: clamp(n, b, a)

  defp clamp(n, a, b) when is_float(n) or is_float(a) or is_float(b),
    do: (n / 1) |> max(a / 1) |> min(b / 1)

  defp clamp(n, a, b), do: n |> max(a) |> min(b)
end
