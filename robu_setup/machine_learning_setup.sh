
source "/pfad/zu/rpi_detect.sh"

# mkdir -p ~/miniconda3
# wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
# bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
# rm ~/miniconda3/miniconda.sh

# source ~/miniconda3/bin/activate

# conda init --all


# #Tests
# conda list
# conda --version


# conda create --name „robu_conda“ python=3.10
 
# conda activate „robu_conda“
 
# conda update -n base -c defaults conda
 
# conda install numpy
 
# conda install pillow
 
# pip install opencv-python
 
# pip install opencv-contrib-python
 
# conda install pyqt
 
# conda install matplotlib
 
# pip install jupyterlab
 
# jupyter lab

if is_raspberry_pi; then
    pip install jupyterlab --break-system-packages
    #pip install notebook --break-system-packages
    pip install --upgrade pip --break-system-packages
    pip install "numpy<2.0"  --break-system-packages
    #pip install opencv-python==4.2.0
    pip install opencv-contrib-python --break-system-packages
    pip install pyqt5 --break-system-packages
    pip install matplotlib --break-system-packages
    pip install keras --break-system-packages
    pip install scipy --break-system-packages
    pip install scikit-learn --break-system-packages
    pip install tensorflow --break-system-packages
    # jupyter lab
fi