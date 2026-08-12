param(
    [string]$PortName,
    [single]$Vx = 0.0,
    [single]$Wz = 0.0,
    [ValidateRange(1, 100)]
    [int]$RateHz = 20,
    [ValidateRange(0.0, 3600.0)]
    [double]$DurationSeconds = 5.0,
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function New-MiniPCChassisCommandFrame {
    param(
        [single]$LinearVelocity,
        [single]$AngularVelocity
    )

    # 创建固定12字节的底盘速度控制帧
    [byte[]]$frame = [byte[]]::new(12)
    $frame[0] = 0x42
    $frame[1] = 0x31
    $frame[2] = 0x0C

    # 按STM32使用的小端序写入单精度浮点数
    [byte[]]$vxBytes = [BitConverter]::GetBytes([single]$LinearVelocity)
    [byte[]]$wzBytes = [BitConverter]::GetBytes([single]$AngularVelocity)
    if (-not [BitConverter]::IsLittleEndian) {
        [Array]::Reverse($vxBytes)
        [Array]::Reverse($wzBytes)
    }

    # 将vx放入第3到第6字节
    [Array]::Copy($vxBytes, 0, $frame, 3, 4)
    # 将wz放入第7到第10字节
    [Array]::Copy($wzBytes, 0, $frame, 7, 4)

    # 累加前11字节并取低8位作为校验和
    [uint32]$checksum = 0
    for ($index = 0; $index -lt 11; $index++) {
        $checksum += $frame[$index]
    }
    $frame[11] = [byte]($checksum -band 0xFF)

    # 返回完整的二进制控制帧
    return $frame
}

# 拒绝向下位机发送非有限浮点数
if ([single]::IsNaN($Vx) -or [single]::IsInfinity($Vx) -or
    [single]::IsNaN($Wz) -or [single]::IsInfinity($Wz)) {
    throw 'Vx and Wz must be finite floating-point values.'
}

# 按当前MiniPC协议生成本次循环发送的数据帧
[byte[]]$commandFrame = New-MiniPCChassisCommandFrame -LinearVelocity $Vx -AngularVelocity $Wz
$frameHex = ($commandFrame | ForEach-Object { $_.ToString('X2') }) -join ' '

Write-Host "Target: vx=$Vx m/s, wz=$Wz rad/s"
Write-Host "Rate: $RateHz Hz"
Write-Host "Duration: $(if ($DurationSeconds -eq 0.0) { 'until Ctrl+C' } else { "$DurationSeconds seconds" })"
Write-Host "Frame: $frameHex"

# 仅生成并检查数据帧时，不访问任何串口
if ($DryRun) {
    Write-Host 'DryRun complete: no serial port was opened and no data was sent.'
    exit 0
}

# 未指定串口时列出当前可用端口并停止
if ([string]::IsNullOrWhiteSpace($PortName)) {
    $availablePorts = [IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    $portText = if ($availablePorts.Count -gt 0) { $availablePorts -join ', ' } else { 'none' }
    throw "Specify -PortName, for example COM7. Available ports: $portText"
}

# 使用USART6当前配置创建115200、8N1串口
$serial = [IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [IO.Ports.Parity]::None,
    8,
    [IO.Ports.StopBits]::One
)
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.WriteTimeout = 500

[uint32]$sentCount = 0
$periodMilliseconds = 1000.0 / $RateHz
$timer = [Diagnostics.Stopwatch]::StartNew()
$nextSendMilliseconds = 0.0

try {
    # 打开指定串口
    $serial.Open()
    Write-Host "Opened $PortName. Sending command frames."

    # 持续发送控制帧，DurationSeconds为0时由Ctrl+C结束
    while (($DurationSeconds -eq 0.0) -or ($timer.Elapsed.TotalSeconds -lt $DurationSeconds)) {
        # 一次写入完整12字节，避免主动拆分控制帧
        $serial.Write($commandFrame, 0, $commandFrame.Length)
        $sentCount++

        # 丢弃下位机持续回传的0x32里程计数据，防止本机接收缓存堆积
        if ($serial.BytesToRead -gt 0) {
            $serial.DiscardInBuffer()
        }

        # 按目标频率等待下一次发送时间
        $nextSendMilliseconds += $periodMilliseconds
        $waitMilliseconds = [int][Math]::Floor($nextSendMilliseconds - $timer.Elapsed.TotalMilliseconds)
        if ($waitMilliseconds -gt 0) {
            Start-Sleep -Milliseconds $waitMilliseconds
        }
    }
}
finally {
    # 无论正常完成还是中途停止，都关闭并释放串口
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
    $timer.Stop()
}

Write-Host "Finished. Sent $sentCount frames."
