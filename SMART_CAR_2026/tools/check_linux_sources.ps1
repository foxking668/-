# Compile-only x86 Linux check from Windows. NOT Loongson linking or hardware validation.
param([string]$Compiler="")
$ErrorActionPreference='Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    if(!$Compiler) {$Compiler=(Join-Path (Get-Location) 'tools/python_compiler/ziglang/zig.exe')}
    New-Item -ItemType Directory -Force build/linux-objects | Out-Null
    $includeArgs=@('-Isrc','-Ilegacy','-Itools/validation_headers/generated')
    foreach($module in @('core','imgproc','videoio','highgui')) {
        $path="tools/validation_headers/opencv-python-4.10.0.84/opencv/modules/$module/include"
        if(!(Test-Path -LiteralPath $path)) {throw 'Run tools/fetch_validation_headers.py first'}
        $includeArgs+=@('-isystem',$path)
    }
    $sources=@('src/config.cpp','src/vision.cpp','src/mission.cpp','src/imu.cpp',
      'src/hardware.cpp','src/hardware_linux.cpp','src/linux_main.cpp',
      'tools/hardware_servo_test.cpp','tools/hardware_pwm_probe.cpp','tools/manual_capture.cpp',
      'tests/hardware_tests.cpp','tests/capture_data_tests.cpp')
    foreach($source in $sources) {
        $output='build/linux-objects/'+[IO.Path]::GetFileNameWithoutExtension($source)+'.o'
        & $Compiler c++ -target x86_64-linux-gnu -std=c++17 -Wall -Wextra -Wpedantic @includeArgs -c $source -o $output
        if($LASTEXITCODE -ne 0) {throw "Compile failed: $source"}
    }
    & $Compiler cc -target x86_64-linux-gnu -c legacy/Contral/PID/PID.c -o build/linux-objects/PID.o
    if($LASTEXITCODE -ne 0) {throw 'PID C compile failed'}
    Write-Output "PASS $($sources.Count+1) Linux translation units (compile only; no target library link)"
} finally {Pop-Location}
