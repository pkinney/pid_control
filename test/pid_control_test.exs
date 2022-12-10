defmodule PIDControlTest do
  use ExUnit.Case

  test "new/1 creates a new PIDControl" do
    assert %PIDControl{} = PIDControl.new()
  end

  test "basic proportional response" do
    pid = PIDControl.new(kp: 0.25) |> PIDControl.step(1.0, 0.0)
    assert pid.output == 0.25
  end

  test "initial derivative response is zero" do
    pid = PIDControl.new(kd: 1.0) |> PIDControl.step(1.0, 0.0)
    assert pid.output == 0.0
  end

  test "initial integral response is based on the t value" do
    pid = PIDControl.new(ki: 0.25, t: 2.0) |> PIDControl.step(1.0, 0.0)
    assert pid.output == 0.5
  end

  test "setting use_system_t depends on the the system clock for a t value" do
    pid = PIDControl.new(ki: 0.25, use_system_t: true) |> PIDControl.step(1.0, 0.0)
    :timer.sleep(100)
    pid = pid |> PIDControl.step(1.0, 0.0)
    assert_in_delta pid.output, 0.025, 0.0025
  end

  test "output_min and output_max clamp the output" do
    pid = PIDControl.new(kp: 0.25, output_min: -0.5) |> PIDControl.step(1.0, 5.0)
    assert pid.output == -0.5

    pid = PIDControl.new(kp: 0.25, output_max: 1.5) |> PIDControl.step(5.0, 0.0)
    assert pid.output == 1.25

    pid = PIDControl.new(kp: 0.25, output_max: 1.1) |> PIDControl.step(5.0, 0.0)
    assert pid.output == 1.1
  end
end
