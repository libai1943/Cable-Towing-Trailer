param([double]$Budget=20)
$ErrorActionPreference='Stop'
$taskTimer=[Diagnostics.Stopwatch]::StartNew()
$taskProcess=Start-Process -FilePath (Join-Path (Get-Location).Path 'ampl.exe') -ArgumentList @('run.run') -WorkingDirectory (Get-Location).Path -WindowStyle Hidden -RedirectStandardOutput 'solver.log' -RedirectStandardError 'solver.stderr' -PassThru
$taskFinished=$taskProcess.WaitForExit([Math]::Max(1,[int](1000*$Budget)))
if(-not $taskFinished){
    # Only the process tree started above belongs to this solver invocation.
    taskkill.exe /PID $taskProcess.Id /T /F | Out-Null
    $taskProcess.WaitForExit()
}
$taskTimer.Stop()
$taskProcess.Refresh()
[ordered]@{budget=$Budget;wallSeconds=$taskTimer.Elapsed.TotalSeconds;timedOut=(-not $taskFinished);exitCode=$taskProcess.ExitCode} | ConvertTo-Json | Set-Content -LiteralPath 'process.json' -Encoding ASCII
exit 0
