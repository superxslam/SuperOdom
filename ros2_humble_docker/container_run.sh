
# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR=$(dirname "$PWD")
USER_ID=$(id -u)
GROUP_ID=$(id -g)


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
PROJECT_DIR="/home/qb/humanoid_slam/src"
DATASET_DIR="/home/qb/humanoid_slam/dataset"

# Allow Docker containers to connect to X11 display
xhost +local:docker

# Remove existing container with the same name if it exists
echo "Checking for existing container: $CONTAINER_NAME"
if docker ps -a --format "table {{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME"
fi

# Launch the nvidia-docker container with optimized RViz support
docker run --privileged -it \
           --runtime=nvidia \
           --gpus all \
           --volume="$PROJECT_DIR:/root/superodom_ws/src" \
           --volume="$DATASET_DIR:/root/data" \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --network=host \
           --ipc=host \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           --env="ROS_DOMAIN_ID=0" \
           --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="LIBGL_ALWAYS_INDIRECT=0" \
           --env="LIBGL_ALWAYS_SOFTWARE=0" \
           --env="MESA_GL_VERSION_OVERRIDE=3.3" \
           --env="MESA_GLSL_VERSION_OVERRIDE=330" \
           --env="__GL_SYNC_TO_VBLANK=0" \
           --env="__GL_THREADED_OPTIMIZATIONS=1" \
           --env="QT_OPENGL_BUGLIST=0" \
           --env="QT_OPENGL_NO_SANITY_CHECK=1" \
           --env="NVIDIA_VISIBLE_DEVICES=all" \
           --env="NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility" \
           "$IMAGE_NAME" /bin/bash
