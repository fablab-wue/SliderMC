# Compile and run host-side unit tests (no Pico hardware).
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root

New-Item -ItemType Directory -Force -Path "test/host" | Out-Null

# Python planner scenario twin (no native toolchain required).
$py = $null
foreach ($c in @("python", "python3", "py")) {
  if (Get-Command $c -ErrorAction SilentlyContinue) { $py = $c; break }
}
if (-not $py) {
  Write-Error "No python on PATH. Install Python 3, then re-run."
  exit 1
}
Write-Host "Running planner scenario sim with $py..."
if ($py -eq "py") {
  & $py -3 -m unittest test.sim.test_planner_scenarios -v
} else {
  & $py -m unittest test.sim.test_planner_scenarios -v
}
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$cxx = $null
foreach ($c in @("g++", "clang++")) {
  if (Get-Command $c -ErrorAction SilentlyContinue) { $cxx = $c; break }
}
if (-not $cxx) {
  Write-Warning "No g++/clang++ on PATH - skipping C++ host tests. Install MSYS2 mingw-w64 or LLVM to run them."
  Write-Host "Python planner scenario tests passed."
  exit 0
}

function Invoke-HostTest($name, $out, $src) {
  Write-Host "Compiling $name with $cxx..."
  & $cxx -std=c++17 -Wall -Wextra -Iinclude -DHOST_TEST -o $out @src
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  Write-Host "Running $name..."
  & $out
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Invoke-HostTest "protocol" "test/host/test_protocol.exe" @(
  "src/protocol/parser.cpp",
  "src/protocol/commands.cpp",
  "src/protocol/verbose.cpp",
  "src/config/config_store.cpp",
  "src/board/littlefs_port.cpp",
  "src/board/gpio.cpp",
  "src/motion/motion_stub.cpp",
  "src/motion/motion_path.cpp",
  "src/motion/motion_diag.cpp",
  "test/host/test_protocol_main.cpp"
)

Invoke-HostTest "planner_math" "test/host/test_planner_math.exe" @(
  "src/motion/planner_math.cpp",
  "test/host/test_planner_math.cpp"
)

Invoke-HostTest "motion_path" "test/host/test_motion_path.exe" @(
  "src/motion/motion_path.cpp",
  "src/motion/motion_stub.cpp",
  "src/config/config_store.cpp",
  "test/host/test_motion_path.cpp"
)

Write-Host "All host tests passed."
