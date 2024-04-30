
param (
    [Switch]$help,
    [Switch]$install
)

$invocationPath = $MyInvocation.MyCommand.Path

Remove-Variable WSL_REQUIRED_CORE_VERSION -force -ErrorAction SilentlyContinue
Remove-Variable WSL_MIN_KERNEL_VERSION -force -ErrorAction SilentlyContinue
Set-Variable WSL_REQUIRED_CORE_VERSION -option ReadOnly -scope Script -value ([String]"2")
Set-Variable WSL_MIN_KERNEL_VERSION -option ReadOnly -scope Script -value ([String]"5.10.16")

Remove-Variable WINGET_MIN_VERSION -force -ErrorAction SilentlyContinue
Set-Variable WINGET_MIN_VERSION -option ReadOnly -scope Script -value ([String]"1.7.10861")

Remove-Variable GIT_MIN_VERSION -force -ErrorAction SilentlyContinue
Set-Variable GIT_MIN_VERSION -option ReadOnly -scope Script -value ([String]"2.33.0")

Remove-Variable PYTHON_MIN_VERSION -force -ErrorAction SilentlyContinue
Set-Variable PYTHON_MIN_VERSION -option ReadOnly -scope Script -value ([String]"3.9.0")

Remove-Variable USBIPD_MIN_VERSION -force -ErrorAction SilentlyContinue
Set-Variable USBIPD_MIN_VERSION -option ReadOnly -scope Script -value ([String]"4.1.0")



function Write-Help {
    Write-Host "Usage: ./win11_setup.ps1 -help | -install"
    Write-Host ""
    Write-Host "-help: print this message"
    Write-Host "-install: will attempt to install rather than notify of missing dependencies. Requires admin"
    Write-Host ""
}

####################
#  WSL2 Utilities  #
####################

function Confirm-wslIsInstalled {
    $wslStatusOutput = @(wsl --status) | Out-String
    $wslStatusOutput = $wslStatusOutput.Replace("`0", "")
    if ($wslStatusOutput.Contains("Default Version")) {
        Write-Host "OK-ACK. WSL is installed"
        return $true
    } else {
        Write-Host "OK-NCK. WSL is not installed"
    }

    return $false
}

function Confirm-wslCoreVersion {
    $wslStatusOutput = @(wsl --status) | Out-String
    $wslStatusOutput = $wslStatusOutput.Replace("`0", "")
    if ($wslStatusOutput -like "*Default Version: $WSL_REQUIRED_CORE_VERSION*") {
        Write-Host "OK-ACK. WSL Core Version is 2."
        return $true
    } else {
        Write-Host "OK-NCK. WSL Core Version is not 2."
    }

    return $false
}

function Confirm-wslKernelVersion {
    $wslStatusOutput = @(wsl --version) | Out-String
    $wslStatusOutput = $wslStatusOutput.Replace("`0", "")
    if ($wslStatusOutput -match "Kernel version\: (\d+\.\d+\.\d+)") {
        $kernelVersion = $Matches.1
        $kernelVersion = $kernelVersion.Trim()
        if ([System.Version]($kernelVersion + ".0") -lt [System.Version]($WSL_MIN_KERNEL_VERSION + ".0")) {
            Write-Host "OK-NCK. WSL Kernel Version is $kernelVersion"
        } else {
            Write-Host "OK-ACK. WSL Kernel Version is $kernelVersion"
            return $true
        }
    } else {
        Write-Host "ERROR. WSL Kernel Version not readable. This shouldn't be possible with a validated wsl install..."
        Write-Host "Abort."
        Exit 1
    }

    return $false
}

############
#  Python  #
############

function Confirm-Python3 {
    if ($null -eq (Get-Command "python" -ErrorAction SilentlyContinue)) {
        return $false
    }

    # ughhh windows please don't alias a message script to a missing program
    # existense of the program is how scripts check for it ya f***
    # on top of that, it sends it to stderr so it's not captured by other default checks
    $output = @(python --version) 2>&1 | Out-String
    $output = ([string]($output)).Replace("`0", "")
    # Write-Host $output

    if (!([String]($output) -match "Python was not found")) {
        $output = @(python --version) | Out-String
        $output = $output.Replace("`0", "")

        $res = [Regex]::Match($output, ".*Python (\d+\.\d+\.\d+).*")
        if (($null -eq $res) -or (!($res.Success))) {
            Write-Host "ERROR. Unkown python version output format."
            return $false
        }

        $detectedVersionString = $res.Groups[1].Value
        $detectedVersionString_WV = $detectedVersionString + ".0"
        $pythonMinVersion_WV = $PYTHON_MIN_VERSION + ".0"
        if (!([System.Version]($detectedVersionString_WV) -lt [System.Version]($pythonMinVersion_WV))) {
            Write-Host "OK-ACK. Python version is $detectedVersionString."
            return $true
        }
    } else {
        Write-Host "OK-NCK. Python not installed. System probably has windows store shim."
    }

    return $false
}

#########
#  Git  #
#########

function Confirm-gitIsInstalled {
    $gitStatusOutput = @(git --version) | Out-String
    $gitStatusOutput = $gitStatusOutput.Replace("`0", "")
    if ($gitStatusOutput -match "git version (\d+\.\d+\.\d+).*") {
        $gitVersion = $Matches.1
        $gitVersion = $gitVersion.Trim()
        if ([System.Version]($gitVersion + ".0") -lt [System.Version]($GIT_MIN_VERSION + ".0")) {
            Write-Host "OK-NCK. Git version is $gitVersion"
        } else {
            Write-Host "OK-ACK. Git version is $gitVersion"
            return $true
        }
    }

    return $false
}

############
#  usbipd  #
############

function Confirm-usbipd {
    if (Get-Command -ErrorAction Ignore -Type Application usbipd) {
        $usbipdStatusOutput = @(usbipd --version) | Out-String
        $usbipdStatusOutput = $usbipdStatusOutput.Replace("`0", "")
        if ($usbipdStatusOutput -match "(\d+\.\d+\.\d+).*") {
            $usbipdVersion = $Matches.1
            $usbipdVersion = $usbipdVersion.Trim()
            if ([System.Version]($usbipdVersion + ".0") -lt [System.Version]($USBIPD_MIN_VERSION + ".0")) {
                Write-Host "OK-NCK. usbipd version is $usbipdVersion"
            } else {
                Write-Host "OK-ACK. usbipd version is $usbipdVersion"
                return $true
            }
        } else {
            Write-Host "ERROR. cannot parse usbipd version"
        }
    } else {
        Write-Host "OK-NCK. usbipd is not installed."
    }

    return $false
}

############
#  WINGET  #
############

function Confirm-winget {
    if (Get-Command -ErrorAction Ignore -Type Application winget) {
        $wingetStatusOutput = @(winget --version) | Out-String
        $wingetStatusOutput = $wingetStatusOutput.Replace("`0", "")
        if ($wingetStatusOutput -match "v(\d+\.\d+\.\d+).*") {
            $wingetVersion = $Matches.1
            $wingetVersion = $wingetVersion.Trim()
            if ([System.Version]($wingetVersion + ".0") -lt [System.Version]($WINGET_MIN_VERSION + ".0")) {
                Write-Host "OK-NCK. Winget version is $wingetVersion"
            } else {
                Write-Host "OK-ACK. Winget version is $wingetVersion"
                return $true
            }
        } else {
            Write-Host "ERROR. cannot parse winget version"
        }
    } else {
        Write-Host "ERROR. winget is not installed."
    }

    return $false
}

function Install-wingetPackage {
    param (
        [string]$PackageName
    )

    $wingetInstallOutput = @(winget install --exact $PackageName) | Out-String

    if ($wingetInstallOutput -match "No package found") {
        Write-Host "ERROR. Package not found."
        return $false
    }
    
    if ($wingetInstallOutput -match "Successfully installed") {
        Write-Host "OK. Installed $PackageName via winget."
        return $true
    }

    if ($wingetInstallOutput -match "No newer package versions are available") {
        Write-Host "OK. $PackageName already installed."
        return $true
    }

    Write-Host $wingetInstallOutput
    Write-Host "ERROR. Unknown install status"

    return $false
}

############
#  Script  #
############

Write-Host "Checking critical verions..."

$anyInstallPending = $false
$canInstallMissingItems = $true
$installWsl2 = $false
$installPython = $false
$installGit = $false
$installUsbipd = $false
$installWinget = $false

if (!(Confirm-wslIsInstalled) -or !(Confirm-wslCoreVersion)) {
    $installWsl2 = $true
    $anyInstallPending = $true
}

if (!(Confirm-wslKernelVersion)) {
    Write-Host "CONFLICT. WSL kernel version is too old. Script cannot resolve."
    $installWsl2 = $false
    $canInstallMissingItems = $false
    $anyInstallPending = $true
}

if (!(Confirm-Python3)) {
    $installPython = $true
    $anyInstallPending = $true
}

if (!(Confirm-gitIsInstalled)) {
    $installGit = $true
    $anyInstallPending = $true
}

if (!(Confirm-usbipd)) {
    $installUsbipd = $true
    $anyInstallPending = $true
}

if (!(Confirm-winget)) {
    $installWinget = $true
    $anyInstallPending = $true
}

Write-Host ""
Write-Host ""

if ($anyInstallPending) {
    Write-Host "There are dependencies to install."
    if ($installWsl2) {
        Write-Host "`tWSL2"
    }

    if ($installPython) {
        Write-Host "`tPython"
    }

    if ($installGit) {
        Write-Host "`tGit"
    }

    if ($installUsbipd) {
        Write-Host "`tUsbipd"
    }

    Write-Host ""
} else {
    Write-Host "All dependencies valid."
    Write-Host ""

    Exit 0
}



if (!($install)) {
    Write-Host "Use the -install flag to install missing items."
    Write-Host ""

    Exit 0
}

##################
#  Install Step  #
##################

$installStepFailed = $false

if ($installWsl2) {
    Write-Host "UNWILLING. There are too many pitfalls to automate this easily."
    Write-Host "Please install a WSL2 Ubuntu LTS distro from the store and re-run the script."
    # NOTE: Will Stuckey has prior work here, but the hypervisor mode failures are esoteric
    # and quite a pain to detect and support for arbitrary users. Make them do this manually.
    Exit 1
}

if ($installWinget) {
    Write-Host "UNABLE. Windows 11 should come with winget."
    Write-Host "Unable to proceed with package installations."
    Exit 1
}

if ($installPython) {
    $success = Install-wingetPackage Python.Python.3.11
    if (!($success)) {
        Write-Host "FAILED. Python install failed."
        $installStepFailed = $true
    }
}

if ($installUsbipd) {
    $success = Install-wingetPackage dorssel.usbipd-win
    if (!($success)) {
        Write-Host "FAILED. usbipd install failed."
        $installStepFailed = $true
    }
}

if ($installGit) {
    $success = Install-wingetPackage Git.Git
    if (!($success)) {
        Write-Host "FAILED. Git install failed."
        $installStepFailed = $true
    }
}


Write-Host ""
Write-Host ""

if ($installStepFailed) {
    Write-Host "ERROR. One or more install steps failed."
    Write-Host ""

    Exit 1
}

Write-Host "OK-ACK. All dependencies installed."
Write-Host "NOTE. You will need to restart your shell."
Write-Host ""

Exit 0

