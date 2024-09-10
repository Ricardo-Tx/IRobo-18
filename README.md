# IRobo-18

## Instalation with Docker (Fast)

Make the bash files executable:
```
chmod +x docker-rebuild.sh; chmod +x docker-run.sh 
```
<br>

Run this command to build/rebuild the container:
```
./docker-rebuild.sh
```
And this command to run the container:
```
./docker-run.sh
```


## Instalation with Docker (Manual)

Run this command to build the docker image once after cloning:
```bash
sudo docker build -t ros-noetic-env .
```
<br>

Check if the image is installed with:
```bash
sudo docker image ls
```
A `ros-noetic-env` REPOSITORY should appear.
<br>
<br>

Run the docker container mounting the repository files:
```bash
sudo docker run -it --name ros-noetic-container -v $(pwd):/app ros-noetic-env
```