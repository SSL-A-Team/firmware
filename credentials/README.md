# Credentials

This repository contains Wifi credentials for the robot.

## A-Team Specific Setup

You need to initialize `src/private_credentials/` by running `./util/ateam-credentials-setup.bash`.

If you are not a member of the A-Team, you can copy `src/public_credentials` to `src/private_credentials` and modify the values, or export `export NO_ATEAM_WIFI_CREDENTIALS=true` to use the public ones (which you can modify to suit your needs). 