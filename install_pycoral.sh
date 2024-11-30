#!/bin/bash

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

sudo apt-get update

sudo apt-get install libedgetpu1-max -y

# Get Script Directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Install required packages
pip install -r $DIR/requirements.txt

# Install the Coral Python API
cd $HOME
mkdir temp
git clone https://github.com/virnik0/pycoral_builds.git
cd pycoral_builds
pip install tflite_runtime-2.18.0-cp312-cp312-linux_x86_64.whl
pip install pycoral-2.0.2-cp312-cp312-linux_x86_64.whl

# Clean up
cd $HOME
rm -rf temp

echo "PyCoral installed successfully"
