#!/bin/bash

# Error if pyenv is not available
if ! [ -x "$(command -v pyenv)" ]; then
  echo "Error: pyenv is not installed." >&2
  exit 1
fi

# Installing python 3.9 on pyenv
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
pyenv activate default

# Get Script Directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Install required packages
pip install -r $DIR/requirements.txt

# Install the Coral Python API
cd $HOME

mkdir temp
cd temp
wget https://github.com/feranick/libedgetpu/releases/download/16.0TF2.17.1-1/libedgetpu1-max_16.0tf2.17.1-1.ubuntu24.04_amd64.deb
wget https://github.com/feranick/pycoral/releases/download/2.0.3TF2.17.1/pycoral-2.0.3-cp312-cp312-linux_x86_64.whl
wget https://github.com/feranick/TFlite-builds/releases/download/v2.17.1/tflite_runtime-2.17.1-cp312-cp312-linux_x86_64.whl

sudp dpkg -i libedgetpu1-max_16.0tf2.17.1-1.ubuntu24.04_amd64.deb
pip install tflite_runtime-2.17.1-cp312-cp312-linux_x86_64.whl
pip install pycoral-2.0.3-cp312-cp312-linux_x86_64.whl

# Clean up
cd $HOME
rm -rf temp

# Cd script dir
cd $DIR
cp 99-edgetpu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "PyCoral installed successfully"
