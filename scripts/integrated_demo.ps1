# ============================================================
# 三库一体化联调启动器 (Windows 侧)
# 同时启动:
#   1. WSL: 三库联调节点 + RViz 机器人模型 (GUI 弹出)
#   2. Windows: 算法库 RGB 可视化窗口 (真机相机 + DIST/NEAR)
# 用法: powershell -ExecutionPolicy Bypass -File integrated_demo.ps1
# ============================================================
$ErrorActionPreference = "Continue"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " 三库一体化联调 - 完整效果" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# ---- 1. 启动 WSL 联调 + RViz (后台) ----
Write-Host "`n[1/2] 启动 WSL 联调 + RViz..." -ForegroundColor Yellow
$wslScript = Get-Content "$PSScriptRoot\integrated_demo.sh" -Raw -Encoding UTF8
$wslScript = $wslScript -replace "`r`n", "`n"
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($wslScript))
Start-Process wsl -ArgumentList "-d", "Ubuntu-26.04", "--", "bash", "-lc", "echo $b64 | base64 -d > /tmp/ide.sh && bash /tmp/ide.sh" -WindowStyle Minimized
Start-Sleep -Seconds 3

# ---- 2. 启动算法库 RGB 可视化 (Windows, 前台弹窗) ----
Write-Host "[2/2] 启动算法库 RGB 可视化窗口..." -ForegroundColor Yellow
$exe = "D:\AndrowsData\mechdog_navigation\build_vs\Release\mechdog_navigation.exe"
$sdkbin = "D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64\bin"
if (Test-Path $exe) {
    $env:PATH = "$sdkbin;$env:PATH"
    Write-Host "启动: $exe --real" -ForegroundColor Green
    & $exe --real
} else {
    Write-Host "[错误] exe 不存在: $exe" -ForegroundColor Red
    Write-Host "  请先: cmake --build D:\AndrowsData\mechdog_navigation\build_vs --config Release --target mechdog_navigation --clean-first"
}

Write-Host "`n一体化联调结束 (RGB 窗口已关闭, WSL 侧已自动清理)" -ForegroundColor Cyan
