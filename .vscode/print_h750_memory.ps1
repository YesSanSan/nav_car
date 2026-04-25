param(
    [Parameter(Mandatory = $true)]
    [string]$WorkspaceFolder,

    [Parameter(Mandatory = $true)]
    [string]$ConfigurePreset
)

$ErrorActionPreference = "Stop"

function Format-Size {
    param([UInt64]$Size)

    if ($Size -eq 0) {
        return "0 B"
    }

    if (($Size % 1KB) -eq 0) {
        return ("{0} KB" -f ($Size / 1KB))
    }

    return ("{0} B" -f $Size)
}

function Get-GlobalLayoutBar {
    param(
        [object[]]$Regions,
        [object]$Region,
        [int]$Width = 48
    )

    $totalLength = ($Regions | Measure-Object -Property Length -Sum).Sum
    if ($totalLength -eq 0) {
        return ""
    }

    $layoutRegions = foreach ($mappedRegion in $Regions) {
        $exactWidth = ([double]$mappedRegion.Length * [double]$Width) / [double]$totalLength
        $baseWidth = [int][Math]::Floor($exactWidth)
        if ($baseWidth -lt 2) {
            $baseWidth = 2
        }

        [PSCustomObject]@{
            Name = $mappedRegion.Name
            Color = $mappedRegion.Color
            Length = $mappedRegion.Length
            Used = $mappedRegion.Used
            ExactWidth = $exactWidth
            Fraction = $exactWidth - [Math]::Floor($exactWidth)
            Cells = $baseWidth
        }
    }

    $assigned = ($layoutRegions | Measure-Object -Property Cells -Sum).Sum
    if ($assigned -lt $Width) {
        $growOrder = $layoutRegions | Sort-Object Fraction -Descending
        $index = 0
        while ($assigned -lt $Width) {
            $growOrder[$index % $growOrder.Count].Cells += 1
            $assigned += 1
            $index += 1
        }
    } elseif ($assigned -gt $Width) {
        $shrinkOrder = $layoutRegions | Sort-Object Fraction
        $index = 0
        while ($assigned -gt $Width) {
            $candidate = $shrinkOrder[$index % $shrinkOrder.Count]
            if ($candidate.Cells -gt 2) {
                $candidate.Cells -= 1
                $assigned -= 1
            }
            $index += 1
        }
    }

    $segments = New-Object System.Collections.Generic.List[string]

    foreach ($mappedRegion in $layoutRegions) {
        $usedCells = 0
        if ($mappedRegion.Name -eq $Region.Name) {
            if ($Region.Used -eq 0) {
                $usedCells = 0
            } else {
                $usedRatio = [double]$Region.Used / [double]$Region.Length
                if ($usedRatio -le 0.5) {
                    $usedCells = 1
                } else {
                    $usedCells = [int][Math]::Round($usedRatio * $mappedRegion.Cells, [System.MidpointRounding]::AwayFromZero)
                    if ($usedCells -lt 1) {
                        $usedCells = 1
                    }
                }
            }
        }

        if ($usedCells -gt $mappedRegion.Cells) {
            $usedCells = $mappedRegion.Cells
        }

        for ($i = 0; $i -lt $mappedRegion.Cells; $i++) {
            $char = if ($i -lt $usedCells) { "█" } else { "·" }
            $segments.Add(("{0}{1}" -f $mappedRegion.Color, $char)) | Out-Null
        }
    }

    while ($segments.Count -gt $Width) {
        $segments.RemoveAt($segments.Count - 1)
    }

    while ($segments.Count -lt $Width) {
        $segments.Add(("{0}·" -f $script:WhiteCode)) | Out-Null
    }

    $segments.Add($script:ResetCode) | Out-Null
    return ($segments -join "")
}

function Get-ColoredName {
    param(
        [string]$Name,
        [string]$ColorCode,
        [int]$Width = 15
    )

    return ("{0}{1}{2}" -f $ColorCode, $Name.PadLeft($Width), $script:ResetCode)
}

function Parse-LinkerRegions {
    param([string]$LinkerScriptPath)

    $content = Get-Content $LinkerScriptPath
    $regions = New-Object System.Collections.Generic.List[object]
    $inMemory = $false

    foreach ($line in $content) {
        if ($line -match '^\s*MEMORY\s*$') {
            $inMemory = $true
            continue
        }

        if (-not $inMemory) {
            continue
        }

        if ($line -match '^\s*}\s*$') {
            break
        }

        if ($line -match '^\s*([A-Za-z0-9_]+)\s*\([^)]+\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)\s*,\s*LENGTH\s*=\s*([0-9A-Za-z]+)') {
            $name = $Matches[1]
            $origin = [UInt64]::Parse($Matches[2].Substring(2), [System.Globalization.NumberStyles]::HexNumber)
            $lengthToken = $Matches[3]

            if ($lengthToken -match '^(\d+)([KkMm])?$') {
                $value = [UInt64]$Matches[1]
                $unit = $Matches[2]
                switch ($unit) {
                    "K" { $length = $value * 1KB; break }
                    "k" { $length = $value * 1KB; break }
                    "M" { $length = $value * 1MB; break }
                    "m" { $length = $value * 1MB; break }
                    default { $length = $value; break }
                }
            } else {
                throw "Unsupported LENGTH token in linker script: $lengthToken"
            }

            $regions.Add([PSCustomObject]@{
                Name = $name
                Origin = $origin
                Length = $length
                End = $origin + $length - 1
            }) | Out-Null
        }
    }

    if ($regions.Count -eq 0) {
        throw "No MEMORY regions found in $LinkerScriptPath"
    }

    return $regions
}

function Parse-AllocatedSections {
    param([string]$ElfPath)

    $objdumpOutput = & arm-none-eabi-objdump -h $ElfPath 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw ("arm-none-eabi-objdump failed for {0}`n{1}" -f $ElfPath, ($objdumpOutput -join [Environment]::NewLine))
    }

    $sections = New-Object System.Collections.Generic.List[object]
    $current = $null

    foreach ($line in $objdumpOutput) {
        if ($line -match '^\s*\d+\s+(\S+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+\S+$') {
            $current = [PSCustomObject]@{
                Name = $Matches[1]
                Size = [UInt64]::Parse($Matches[2], [System.Globalization.NumberStyles]::HexNumber)
                Vma = [UInt64]::Parse($Matches[3], [System.Globalization.NumberStyles]::HexNumber)
                Lma = [UInt64]::Parse($Matches[4], [System.Globalization.NumberStyles]::HexNumber)
                Flags = ""
            }
            continue
        }

        if ($null -ne $current -and $line -match '^\s{18}(.+)$') {
            $flags = $Matches[1].Trim()
            $current | Add-Member -NotePropertyName Flags -NotePropertyValue $flags -Force
            if ($flags -match '\bALLOC\b' -and $current.Size -gt 0) {
                $sections.Add($current) | Out-Null
            }
            $current = $null
        }
    }

    return $sections
}

$overlayDir = Join-Path $WorkspaceFolder "build/h750_boot_overlay"
$presetDir = Join-Path $overlayDir $ConfigurePreset
$buildDir = Join-Path (Join-Path $presetDir "build") $ConfigurePreset
$elfPath = Join-Path $buildDir "nav_car.elf"
$linkerScriptPath = Join-Path $presetDir "STM32H750XX_FLASH.ld"

if (-not (Test-Path -LiteralPath $elfPath)) {
    throw "ELF not found: $elfPath"
}

if (-not (Test-Path -LiteralPath $linkerScriptPath)) {
    $linkerScriptPath = Join-Path $WorkspaceFolder "STM32H750XX_FLASH.ld"
}

if (-not (Test-Path -LiteralPath $linkerScriptPath)) {
    throw "Linker script not found: $linkerScriptPath"
}

$regions = Parse-LinkerRegions -LinkerScriptPath $linkerScriptPath
$sections = Parse-AllocatedSections -ElfPath $elfPath

$esc = [char]27
$script:ResetCode = "$esc[0m"
$script:WhiteCode = "$esc[37m"

$palette = @(
    "$esc[38;5;39m",
    "$esc[38;5;42m",
    "$esc[38;5;214m",
    "$esc[38;5;201m",
    "$esc[38;5;45m",
    "$esc[38;5;220m",
    "$esc[38;5;141m",
    "$esc[38;5;82m"
)

$regionInfos = foreach ($index in 0..($regions.Count - 1)) {
    $region = $regions[$index]
    $used = [UInt64]0

    foreach ($section in $sections) {
        if ($section.Vma -ge $region.Origin -and $section.Vma -lt ($region.Origin + $region.Length)) {
            $used += $section.Size
        }

        if ($section.Lma -ne $section.Vma -and $section.Flags -match '\bLOAD\b' -and $section.Lma -ge $region.Origin -and $section.Lma -lt ($region.Origin + $region.Length)) {
            $used += $section.Size
        }
    }

    [PSCustomObject]@{
        Name = $region.Name
        Origin = $region.Origin
        End = $region.End
        Length = $region.Length
        Used = $used
        Percent = if ($region.Length -eq 0) { 0.0 } else { [double]$used * 100.0 / [double]$region.Length }
        Color = $palette[$index % $palette.Count]
    }
}

Write-Host ""
Write-Host "Memory region         Used Size  Region Size  %age Used         Start          End  Layout"
foreach ($region in $regionInfos) {
    $name = Get-ColoredName -Name ($region.Name + ":") -ColorCode $region.Color
    $used = (Format-Size $region.Used).PadLeft(12)
    $size = (Format-Size $region.Length).PadLeft(11)
    $percent = ("{0,9:N2}%" -f $region.Percent)
    $start = ("0x{0:X8}" -f $region.Origin).PadLeft(12)
    $end = ("0x{0:X8}" -f $region.End).PadLeft(12)
    $bar = Get-GlobalLayoutBar -Regions $regionInfos -Region $region
    Write-Host ("{0}  {1}  {2}  {3}  {4}  {5}  {6}" -f $name, $used, $size, $percent, $start, $end, $bar)
}
