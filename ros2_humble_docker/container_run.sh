
# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR=$(dirname "$PWD")

# Move to the parent folder of the project directory
cd "$PROJECT_DIR"

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# Check if arguments are provided for the image name and tag
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <container_name> <image_name:tag>"
  exit 1
fi

# Assign the arguments to variables for clarity
CONTAINER_NAME="$1"
IMAGE_NAME="$2"
PROJECT_DIR="/home/shibo/project_workspaces/SuperOdom"
DATASET_DIR="/home/shibo/project_workspaces/datasets"

# Allow Docker containers to connect to X11 display
xhost +local:docker

# Remove existing container with the same name if it exists
echo "Checking for existing container: $CONTAINER_NAME"
if docker ps -a --format "table {{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME"
fi

# Launch the nvidia-docker container with the provided image name and tag
docker run --privileged -it \
           --volume="$PROJECT_DIR:/root/ros2_ws/src" \
           --volume="$DATASET_DIR:/root/data" \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=4gb \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           "$IMAGE_NAME" /bin/bash
