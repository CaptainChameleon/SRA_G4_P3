param (
    [Alias("r")][switch]$Run,
    [Alias("p")][switch]$Push,
    [Alias("l")][switch]$Logs
)
$single_commands = $Run -or $Push -or $Logs


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
    Write-Host "ERROR: Please run this script from the 'SRA_G4_P3' folder" -ForegroundColor Red
    exit 1
}
if (-not (Test-Connection -ComputerName ev3dev -Count 1 -Quiet))
{
    Write-Host "ERROR: EV3DEV is not reachable." -ForegroundColor Red
    exit 1
}
if (-not (Test-Path -Path "./logs"))
{
    mkdir ./logs
}


$password = "maker"
if (-not $single_commands -or ($single_commands -and $Push))
{
    # PUSH SOURCE CODE TO EV3DEV
    Write-Host "Pushing code to EV3DEV..." -ForegroundColor Green
    plink -batch -ssh robot@ev3dev -pw $password "rm -r /home/robot/SRA_G4_P3/*"
    pscp -batch -r -pw $password ./src robot@ev3dev:/home/robot/SRA_G4_P3
    pscp -batch -pw $password ./config.ini robot@ev3dev:/home/robot/SRA_G4_P3
    plink -batch -ssh robot@ev3dev -pw $password "chmod +rx /home/robot/SRA_G4_P3/src/*.py"
}
if (-not $single_commands -or ($single_commands -and $Run))
{
    # LAUNCH SCRIPT WITHIN EV3DEV
    Write-Host "Launching EV3DEV program..." -ForegroundColor Green
    plink -batch -ssh robot@ev3dev -pw $password "python3 /home/robot/SRA_G4_P3/src/main_p3.py"
    # plink -batch -ssh robot@ev3dev -pw $password "python3 /home/robot/SRA_G4_P3/src/main_trabajo_final.py"
}
if (-not $single_commands -or ($single_commands -and $Logs))
{
    # GET LOGS
    Write-Host "Downloading logs..." -ForegroundColor Green
    pscp -batch -pw $password robot@ev3dev:/home/robot/SRA_G4_P3/*.log ./logs/
    pscp -batch -pw $password robot@ev3dev:/home/robot/SRA_G4_P3/src/*.log ./logs/
}

Write-Host "Done!" -ForegroundColor Green