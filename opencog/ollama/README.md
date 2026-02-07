

Hints and Notes
===============

Run
---
```
docker start -i aol
./run-tmux.sh
ollama serve &
ollama list
ollama run qwen3:8b
```

Docker wont start
-----------------
To fix the error message below:

```
docker start -i aol
Error response from daemon: could not select device driver "" with capabilities: [[gpu]]
```
Solution:

Is `nvidia-ctk` installed?
```
nvidia-ctk --version
```
If not, get the keyring, install.
```
wget -qO- https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

wget -qO- https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install nvidia-container-toolkit
```
Then
```
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
