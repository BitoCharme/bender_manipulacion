cpu_flag=false

while getopts ":c" flag; do
  case $flag in
    c) 
      cpu_flag=true;;
    *)
      echo "Unknown option -$OPTARG."
      exit 1 ;;
  esac
done

if $cpu_flag; then
  echo "CPU"
  docker build --build-arg="BASE_OS=ubuntu:24.04" -t ros2_jazzy_devel .
else 
  echo "GPU"
  docker build -t ros2_jazzy_devel .
fi
