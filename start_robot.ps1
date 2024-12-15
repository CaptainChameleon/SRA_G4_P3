param (
    [Alias("r")][switch]$RunOnly,
    [Alias("p")][switch]$PushOnly
)
if ($RunOnly -and $PushOnly) {
    Write-Host "ERROR: You cannot use both -RunOnly and -PushOnly at the same time." -ForegroundColor Red
    exit 1
}


function Check-Command($cmdname)
{
    return [bool](Get-Command -Name $cmdname -ErrorAction SilentlyContinue)
}
if (-not (Check-Command -cmdname 'plink'))
{
     Write-Host "PuTTy not found. Installing..."
     winget install PuTTY.PuTTY --disable-interactivity
}


$currentPath = Get-Location
if (-not ($currentPath.path.EndsWith("SRA_G4_P3")))
{
    Write-Host "ERROR: Please run this script from the 'SRA_G4_P3' folder"
    exit 1
}
if (-not (Test-Connection -ComputerName ev3dev -Count 1 -Quiet))
{
    Write-Host "ERROR: EV3DEV is not reachable."
    exit 1
}
if (-not (Test-Path -Path "./logs"))
{
    mkdir ./logs
}


$password = "maker"
if (-not $RunOnly)
{
    # PUSH SOURCE CODE TO EV3DEV
    Write-Host "Pushing code to EV3DEV..."
    $ev3dev_launch_script = "rm -r /home/robot/SRA_G4_P3/*"
    plink -batch -ssh robot@ev3dev -pw $password $ev3dev_launch_script
    pscp -batch -r -pw $password ./src robot@ev3dev:/home/robot/SRA_G4_P3
    $ev3dev_launch_script = "chmod +rx /home/robot/SRA_G4_P3/src/*.py"
    plink -batch -ssh robot@ev3dev -pw $password $ev3dev_launch_script
}
if (-not $PushOnly)
{
    # LAUNCH SCRIPT WITHIN EV3DEV
    Write-Host "Launching EV3DEV program..."
    $ev3dev_launch_script = "python3 /home/robot/SRA_G4_P3/src/main.py"
    plink -batch -ssh robot@ev3dev -pw $password $ev3dev_launch_script

    # GET LOGS
    Write-Host "Downloading logs..."
    pscp -batch -pw $password robot@ev3dev:/home/robot/SRA_G4_P3/*.log ./logs/
}

Write-Host "Done!"