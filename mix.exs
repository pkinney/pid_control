defmodule PIDControl.MixProject do
  use Mix.Project

  def project do
    [
      app: :pid_control,
      version: "0.1.0",
      elixir: "~> 1.14",
      start_permanent: Mix.env() == :prod,
      aliases: aliases(),
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:telemetry, "~> 1.0"},
      {:credo, "~> 1.6.5", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.0", only: [:dev], runtime: false}
    ]
  end

  defp aliases do
    [
      validate: [
        "clean",
        "compile --warnings-as-errors",
        "format --check-formatted",
        "credo",
        "dialyzer"
      ]
    ]
  end
end