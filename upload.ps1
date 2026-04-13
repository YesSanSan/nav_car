$maxRetry = 5
$attempt = 0

while ($attempt -lt $maxRetry) {
    Write-Host "Task attempt $($attempt + 1)"
    openocd -f 'D:/scoop/apps/openocd-esp/current/openocd-esp32/share/openocd/scripts/interface/cmsis-dap.cfg' -f 'D:/scoop/apps/openocd-esp/current/openocd-esp32/share/openocd/scripts/target/stm32h7x.cfg' -c 'adapter speed 1000; init; halt; if {[catch {reset halt}]} { echo {reset failed, trying force halt}; halt; }; sleep 500; program build/nav_car.elf verify; reset run; exit'

    if ($LASTEXITCODE -eq 0) {
        Write-Host "Task succeeded"
        exit 0
    }

    $attempt++
    Write-Host "Task failed, retrying..."
}

Write-Error "Task failed after $maxRetry attempts"
exit 1

