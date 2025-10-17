#!/bin/bash

# https://www.robotis.com/service/download.php?no=1671

sudo chmod 775 DynamixelWizard2Setup_x64

./DynamixelWizard2Setup_x64
sudo usermod -aG dialout $USER