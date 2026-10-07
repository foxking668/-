param([string]$Compiler="",[switch]$SkipTests)
$ErrorActionPreference="Stop"
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    if (!$Compiler) {
        $bundled=Join-Path $PSScriptRoot 'tools/zig-windows-x86_64-0.13.0/zig.exe'
        $pythonCompiler=Join-Path $PSScriptRoot 'tools/python_compiler/ziglang/zig.exe'
        if (Test-Path -LiteralPath $bundled) {$Compiler=$bundled}
        elseif (Test-Path -LiteralPath $pythonCompiler) {$Compiler=$pythonCompiler}
        elseif (Get-Command g++ -ErrorAction SilentlyContinue) {$Compiler='g++'}
        else {throw 'Need Zig 0.13 or g++; use -Compiler with the executable path.'}
    }
    $prefix=@()
    if ([IO.Path]::GetFileNameWithoutExtension($Compiler) -eq 'zig') {$prefix=@('c++')}
    $common=@('-std=c++17','-Wall','-Wextra','-Wpedantic','-O0','-g','-Isrc','src/config.cpp','src/vision.cpp','src/mission.cpp','src/imu.cpp','src/visual_observer.cpp')
    & $Compiler @prefix @common 'tests/core_tests.cpp' '-o' 'build/core_tests.exe'
    if($LASTEXITCODE -ne 0) {throw 'Core tests compilation failed'}
    & $Compiler @prefix @common 'src/replay_stream.cpp' '-o' 'build/vision_stream.exe'
    if($LASTEXITCODE -ne 0) {throw 'Replay compilation failed'}
    & $Compiler @prefix @common 'tests/visual_observer_tests.cpp' '-o' 'build/visual_observer_tests.exe'
    if($LASTEXITCODE -ne 0) {throw 'Visual observer tests compilation failed'}
    & $Compiler @prefix @common 'src/hardware.cpp' 'tests/manual_steering_tests.cpp' '-o' 'build/manual_steering_tests.exe'
    if($LASTEXITCODE -ne 0) {throw 'Manual steering tests compilation failed'}
    & $Compiler @prefix '-std=c++17' '-Wall' '-Wextra' '-Wpedantic' '-O0' '-g' '-Isrc' 'src/config.cpp' 'src/hardware.cpp' 'tests/hardware_tests.cpp' '-o' 'build/hardware_tests.exe'
    if($LASTEXITCODE -ne 0) {throw 'Hardware tests compilation failed'}
    & $Compiler @prefix '-std=c++17' '-Wall' '-Wextra' '-Wpedantic' '-O0' '-g' '-Isrc' 'tests/capture_data_tests.cpp' '-o' 'build/capture_data_tests.exe'
    if($LASTEXITCODE -ne 0) {throw 'Capture-data tests compilation failed'}
    if(!$SkipTests) {
        & './build/core_tests.exe' 'config/competition.ini'
        if($LASTEXITCODE -ne 0) {throw 'Core regression tests failed'}
        & './build/visual_observer_tests.exe'
        if($LASTEXITCODE -ne 0) {throw 'Visual observer tests failed'}
        & './build/manual_steering_tests.exe'
        if($LASTEXITCODE -ne 0) {throw 'Manual steering tests failed'}
        & './build/hardware_tests.exe' 'config/hardware.ini'
        if($LASTEXITCODE -ne 0) {throw 'Hardware regression tests failed'}
        & './build/capture_data_tests.exe'
        if($LASTEXITCODE -ne 0) {throw 'Capture-data tests failed'}
    }
} finally {Pop-Location}
