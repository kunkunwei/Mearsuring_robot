param(
    [string]$PortName = 'COM12',
    [ValidateRange(-0.2, 0.2)]
    [single]$TestVx = 0.10,
    [ValidateRange(1, 50)]
    [int]$RateHz = 20,
    [ValidateRange(1.0, 10.0)]
    [double]$WarmupSeconds = 3.0,
    [ValidateRange(0.5, 5.0)]
    [double]$MotionSeconds = 2.0,
    [ValidateRange(1.0, 10.0)]
    [double]$StopSeconds = 2.0,
    [switch]$ChassisLifted,
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

    # 将速度值复制到协议规定的位置
    [Array]::Copy($vxBytes, 0, $frame, 3, 4)
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

function Format-HexFrame {
    param([byte[]]$Frame)

    # 将二进制帧转换为便于人工检查的十六进制文本
    return ($Frame | ForEach-Object { $_.ToString('X2') }) -join ' '
}

function Send-CommandPhase {
    param(
        [IO.Ports.SerialPort]$Serial,
        [string]$Name,
        [byte[]]$Frame,
        [double]$Seconds,
        [int]$FrequencyHz
    )

    # 根据持续时间和频率计算本阶段发送帧数
    $frameCount = [int][Math]::Ceiling($Seconds * $FrequencyHz)
    $periodMilliseconds = [int][Math]::Max(1, [Math]::Round(1000.0 / $FrequencyHz))
    Write-Host "$Name ($Seconds s): $(Format-HexFrame -Frame $Frame)"

    # 按固定周期发送完整控制帧
    for ($frameIndex = 0; $frameIndex -lt $frameCount; $frameIndex++) {
        $Serial.Write($Frame, 0, $Frame.Length)

        # 丢弃下位机持续回传的里程计数据，避免本机接收缓存堆积
        if ($Serial.BytesToRead -gt 0) {
            $Serial.DiscardInBuffer()
        }

        Start-Sleep -Milliseconds $periodMilliseconds
    }
}

# 生成准备、动作和停止三个阶段的数据帧
[byte[]]$zeroFrame = New-MiniPCChassisCommandFrame -LinearVelocity 0.0 -AngularVelocity 0.0
[byte[]]$motionFrame = New-MiniPCChassisCommandFrame -LinearVelocity $TestVx -AngularVelocity 0.0

Write-Host "Port: $PortName"
Write-Host "Motion target: vx=$TestVx m/s, wz=0 rad/s"
Write-Host "Sequence: stop $WarmupSeconds s -> move $MotionSeconds s -> stop $StopSeconds s"
Write-Host "Zero frame: $(Format-HexFrame -Frame $zeroFrame)"
Write-Host "Motion frame: $(Format-HexFrame -Frame $motionFrame)"

# 仅检查封包时，不打开串口也不要求安全确认
if ($DryRun) {
    Write-Host 'DryRun complete: no serial port was opened and no data was sent.'
    exit 0
}

# 实际动作测试必须由用户确认底盘已经架空
if (-not $ChassisLifted) {
    throw 'Refusing motion test. Lift the chassis and add -ChassisLifted.'
}

# 创建与USART6一致的115200、8N1串口
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

try {
    # 打开串口并按安全顺序执行动作测试
    $serial.Open()
    Write-Host 'Opened serial port. Keep hands away from the wheels.'
    Send-CommandPhase -Serial $serial -Name 'PREPARE/STOP' -Frame $zeroFrame -Seconds $WarmupSeconds -FrequencyHz $RateHz
    Send-CommandPhase -Serial $serial -Name 'FORWARD' -Frame $motionFrame -Seconds $MotionSeconds -FrequencyHz $RateHz
    Send-CommandPhase -Serial $serial -Name 'FINAL STOP' -Frame $zeroFrame -Seconds $StopSeconds -FrequencyHz $RateHz
}
finally {
    # 无论正常结束还是发生异常，都尽量补发一秒停止指令
    if ($serial.IsOpen) {
        try {
            Send-CommandPhase -Serial $serial -Name 'SAFETY STOP' -Frame $zeroFrame -Seconds 1.0 -FrequencyHz $RateHz
        }
        catch {
            Write-Warning 'Unable to send the final safety-stop frames.'
        }
        $serial.Close()
    }
    $serial.Dispose()
}

Write-Host 'Motion test completed and stop frames were sent.'
