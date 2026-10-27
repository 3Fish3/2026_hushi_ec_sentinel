# 仅用于无执行器台架的达妙 USB2CAN 串口测试, 不适用于 USB2CANFD SDK.
# 依据官方 usb-2-can 仓库 19255a0 的协议表及 USB2CAN_2.0.0.3.exe.
param(
    [string]$Port,
    [switch]$NoActuators,
    [ValidateRange(1,5000)][int]$Count=1,
    [ValidateRange(1,1000)][int]$IntervalMs=10,
    [ValidateSet(0x321,0x208)][int]$CanId=0x321,
    [ValidateSet(0,1)][int]$Extended=0,
    [ValidateSet(0,1)][int]$Remote=0,
    [ValidateRange(0,8)][int]$Dlc=8
)
$ErrorActionPreference='Stop'
if (-not $NoActuators -or [string]::IsNullOrWhiteSpace($Port)) {
    throw 'Specify -Port and -NoActuators only after confirming an isolated bench without actuators'
}
if ($Count*$IntervalMs -gt 30000) { throw 'Requested burst exceeds the 30 second budget' }

# 不扫描端口、不改波特率配置、不写持久参数、不发送电机控制 ID.
# CAN 速率须事先确认为 1 Mbps; 本脚本只设置 PC 到模块的串口参数.
$portHandle=[System.IO.Ports.SerialPort]::new($Port,921600,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$portHandle.ReadTimeout=500
$portHandle.WriteTimeout=1000
$portHandle.DtrEnable=$false
$portHandle.RtsEnable=$false
$portHandle.Handshake=[System.IO.Ports.Handshake]::None
$bytes=[Collections.Generic.List[byte]]::new()
try {
    $portHandle.Open()
    $portHandle.RtsEnable=$true
    $portHandle.DiscardInBuffer()
    [byte[]]$frame=0x55,0xaa,0x1e,0x01,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0x53,0x31,0x43,0x41,0x4e,0x01,0xa5,0x5a,0
    [BitConverter]::GetBytes([uint32]$Count).CopyTo($frame,4)
    [BitConverter]::GetBytes([uint32]($IntervalMs*10)).CopyTo($frame,8)
    $frame[12]=[byte]$Extended
    [BitConverter]::GetBytes([uint32]$CanId).CopyTo($frame,13)
    $frame[17]=[byte]$Remote
    $frame[18]=[byte]$Dlc
    # 合成 GM6020 反馈: 编码器 4096, 1000 rpm, 原始电流 123, 温度 42 C.
    if ($CanId -eq 0x208) { ([byte[]](0x10,0,0x03,0xe8,0,0x7b,42,0)).CopyTo($frame,21) }
    # 与当前官方程序一致: CRC-8 反射多项式 0x8c, 覆盖含零占位 CRC 的完整 30 B.
    [int]$crc=0
    foreach ($b in $frame) {
        $crc=$crc -bxor $b
        for ($i=0;$i -lt 8;$i++) {
            if ($crc -band 1) { $crc=($crc -shr 1) -bxor 0x8c } else { $crc=$crc -shr 1 }
        }
    }
    $frame[29]=[byte]$crc
    Write-Output ('TX '+[Convert]::ToHexString($frame))
    $portHandle.Write($frame,0,30)
    $watch=[Diagnostics.Stopwatch]::StartNew()
    $success=0; $failure=0; $errors=0; $parsed=0
    while ($watch.ElapsedMilliseconds -lt ($Count*$IntervalMs+1500)) {
        $n=$portHandle.BytesToRead
        if ($n -gt 0) {
            $buffer=[byte[]]::new($n)
            $got=$portHandle.Read($buffer,0,$n)
            for ($i=0;$i -lt $got;$i++) { $bytes.Add($buffer[$i]) }
            # 有界台架不应产生大量无关数据; 异常时停止而非无限积累.
            if ($bytes.Count -gt 1048576) { throw 'Unexpected serial traffic exceeds the receive budget' }
            while ($parsed+15 -lt $bytes.Count) {
                if ($bytes[$parsed] -ne 0xaa -or $bytes[$parsed+15] -ne 0x55) { $parsed++; continue }
                switch ($bytes[$parsed+1]) {
                    0x12 {
                        $reply=$bytes.GetRange($parsed,16).ToArray()
                        if ([BitConverter]::ToUInt32($reply,3) -ne $CanId -or
                            $reply[2] -ne ($Dlc -bor ($Extended -shl 6) -bor ($Remote -shl 7))) {
                            throw 'TX reply identity or frame format mismatch'
                        }
                        # DLC 外的内容及远程帧载荷无效, 不参与比较.
                        if (-not $Remote) {
                            for ($j=0;$j -lt $Dlc;$j++) {
                                if ($reply[7+$j] -ne $frame[21+$j]) { throw 'TX reply payload mismatch' }
                            }
                        }
                        $success++
                    }
                    0x02 { $failure++ }
                    0xee { $errors++ }
                }
                $parsed+=16
            }
            if ($failure -gt 0 -or $errors -gt 0 -or $success -gt $Count) { break }
        }
        Start-Sleep -Milliseconds 5
    }
    [pscustomobject]@{
        requested=$Count; success=$success; failure=$failure; errors=$errors
        reply_bytes=$bytes.Count; elapsed_ms=$watch.ElapsedMilliseconds
        first_reply=$(if ($bytes.Count -ge 16) { [Convert]::ToHexString($bytes.GetRange(0,16).ToArray()) } else { [Convert]::ToHexString($bytes.ToArray()) })
    } | ConvertTo-Json -Compress
    if ($failure -ne 0 -or $errors -ne 0 -or $success -ne $Count) {
        throw 'CAN transmission not confirmed; pause for supply/adapter investigation, do not auto-retry'
    }
    # 模块发送成功回执不证明板端应用已消费; 须另核对 MCU 接收与丢弃增量.
} finally {
    if ($portHandle.IsOpen) {
        try {
            # 尽力停止模块周期发送; 此命令不是物理动力切断措施.
            [byte[]]$stop=0x55,0x03,0xaa,0x55
            $portHandle.Write($stop,0,4)
        } finally { $portHandle.Close() }
    }
    $portHandle.Dispose()
}
