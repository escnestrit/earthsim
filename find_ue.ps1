# Prints the folder of the newest installed Unreal Engine 5 (Epic Launcher records installs in LauncherInstalled.dat)
$dat = "$env:ProgramData\Epic\UnrealEngineLauncher\LauncherInstalled.dat"
$best = $null
if (Test-Path $dat) {
  $j = Get-Content $dat -Raw | ConvertFrom-Json
  foreach ($i in $j.InstallationList) {
    if ($i.AppName -like "UE_5*" -and (Test-Path "$($i.InstallLocation)\Engine\Build\BatchFiles\Build.bat")) {
      if ($best -eq $null -or $i.AppName -gt $best.AppName) { $best = $i }
    }
  }
}
if ($best) { Write-Output $best.InstallLocation; exit 0 }
foreach ($v in "5.7","5.6","5.5","5.4") {
  foreach ($root in "C:\Program Files\Epic Games","D:\Program Files\Epic Games","D:\Epic Games","E:\Epic Games") {
    if (Test-Path "$root\UE_$v\Engine\Build\BatchFiles\Build.bat") { Write-Output "$root\UE_$v"; exit 0 }
  }
}
exit 1
