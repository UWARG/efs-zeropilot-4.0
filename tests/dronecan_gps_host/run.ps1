$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $output = Join-Path $env:TEMP 'zp-gps-host-test.exe'
    & g++ '-std=c++17' '-include' 'tests/dronecan_gps_host/stubs/host_time.hpp' '-Itests/dronecan_gps_host/stubs' '-Istm32l552xx/boardfiles/drivers/gps' '-Izeropilot4.0/include/driver_ifaces' '-Izeropilot4.0/include/zp_error' tests/dronecan_gps_host/test.cpp stm32l552xx/boardfiles/drivers/gps/dronecan_gps.cpp -o $output
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & $output
    if ($LASTEXITCODE -ne 0) { throw 'GPS behavior test failed' }
} finally { Pop-Location }
