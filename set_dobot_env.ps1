param(
  [string]$PythonExe = ""
)
$dobot = "E:\DualArm\DobotMagician"
if (!(Test-Path $dobot)) {
  Write-Host "DobotMagician 目录不存在: $dobot"
  exit 1
}
if ($env:PYTHONPATH) {
  $env:PYTHONPATH = "$dobot;$env:PYTHONPATH"
} else {
  $env:PYTHONPATH = $dobot
}
$env:Path = "$dobot;$env:Path"
Write-Host "已设置 PYTHONPATH 和 PATH 指向 $dobot"
if ($PythonExe) {
  Write-Host "测试导入 DobotDllType..."
  & $PythonExe -c "import DobotDllType as dType; print('DobotDllType 导入成功')"
}
