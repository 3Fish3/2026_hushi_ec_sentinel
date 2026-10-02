param([Parameter(Mandatory)][string]$CubeMxRoot)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$project = Join-Path $PSScriptRoot 'cubemx/sentinel_s1'
# 生成器只允许写入本板的生成目录; 防止子目录链接把覆盖范围引向外部.
foreach ($p in @((Get-Item -LiteralPath $PSScriptRoot), (Get-Item -LiteralPath (Join-Path $PSScriptRoot 'cubemx')), (Get-Item -LiteralPath $project)) + @(Get-ChildItem -LiteralPath $project -Recurse -Force)) {
    if ($p.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Unexpected reparse point: $($p.FullName)" }
}
$java = Join-Path $CubeMxRoot 'jre/bin/java.exe'
$mx = Join-Path $CubeMxRoot 'STM32CubeMX.exe'
foreach ($p in @($java, $mx)) { if (!(Test-Path -LiteralPath $p)) { throw "Missing tool: $p" } }
$work = Join-Path $repo 'build/cubemx'
New-Item -ItemType Directory -Path $work -Force | Out-Null
$script = Join-Path $work 's1-generate.txt'
$log = Join-Path $work 's1-generate.log'
$ioc = (Join-Path $project 'sentinel_s1.ioc').Replace('\','/')
$parent = (Join-Path $PSScriptRoot 'cubemx').Replace('\','/')
@("config load `"$ioc`"", "project path `"$parent`"", 'project generate', 'exit') | Set-Content -LiteralPath $script -Encoding utf8NoBOM
& $java -jar $mx -q $script *> $log
if ($LASTEXITCODE -ne 0) { throw "CubeMX failed; see $log" }
$text = Get-Content -LiteralPath $log -Raw
if ($text -match 'IP not ready for code generation') { throw "Unresolved CubeMX configuration; see $log" }
$errors = Select-String -LiteralPath $log -Pattern '\[ERROR\]'
foreach ($errorLine in $errors) {
    # 本机 6.18.1 的 CMake 模板存在路径重复问题, 仅影响未采用的 libc I/O 模板.
    # AdobePublisher 是工具遥测, 不属于固件生成. 其他错误必须调查.
    if ($errorLine.Line -notmatch 'AdobePublisher|FileStreamsReadnWrite:.*[\\/](sysmem|syscalls)\.c') {
        throw "Unexpected generator error: $($errorLine.Line)"
    }
}
# CubeMX 对内容未变的文件保留修改时间, 不能用 mtime 判断是否执行生成.
if ($text -notmatch '(?ms)^project generate\r?$.*?^OK\r?$' -or !(Test-Path -LiteralPath (Join-Path $project 'Core/Src/main.c'))) {
    throw 'CubeMX did not report generation completion; do not accept stale output'
}
if ($text -match 'FileStreamsReadnWrite:') { Write-Warning 'Inspect template errors in the log; sysmem/syscalls and generated CMake are not used by the root build.' }
Write-Output "Generated S1 sources. Inspect USER CODE differences and rebuild before accepting: $log"
