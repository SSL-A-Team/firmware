$scriptFilePath = $MyInvocation.MyCommand.Path
$scriptDirPath = (Get-Item $scriptFilePath).DirectoryName

function Confirm-AdministatorPrivledgesActive {
    return ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] 'Administrator')
}

if (!(Confirm-AdministatorPrivledgesActive)) {
    # user has not deliberately run the script as admin
    # print text warning and pause so they know a UAC prompt is coming
    Write-Host "Elevating to administrator runtime. You will prompted via UAC."
    # Write-Host "Press any *LETTER* key to continue..."
    # $null = [Console]::ReadKey() | Out-Null
    
    # elevate
    if ([int](Get-CimInstance -Class Win32_OperatingSystem | Select-Object -ExpandProperty BuildNumber) -ge 6000) {
        Start-Process -FilePath PowerShell.exe -Verb Runas -ArgumentList $scriptFilePath

        Exit 0
    } else {
        # this should only happen on like old Win 7/Vista machines... ether way WSL aint happening
        Write-Host "Operating system build number is severely outdated. Install the latest patch set for windows 10 or 11. Unable to continue."
        Exit 1
    }
}

Start-Process -FilePath python -Verb Runas -ArgumentList "$scriptDirPath/win11_usb_daemon.py"
