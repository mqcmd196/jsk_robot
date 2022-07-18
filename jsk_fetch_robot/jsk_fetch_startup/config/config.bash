# This file is bash configuration for robot.conf
# This file must be at /var/lib/robot/config.bash

if [ $(hostname) = 'fetch15' ]; then
  export DEFAULT_SPEAKER=13;
  export DEFAULT_ENGLISH_SPEAKER=cmu_us_bdl.flitevox;
  export DEFAULT_WARNING_SPEAKER=cmu_us_fem.flitevox;

  export USE_BASE_CAMERA_MOUNT=true;
  export USE_HEAD_BOX=false;
  export USE_HEAD_L515=true;
  export USE_INSTA360_STAND=true;

  export RS_SERIAL_NO_T265="925122110450";
  export RS_SERIAL_NO_D435_FRONTRIGHT="";
  export RS_SERIAL_NO_D435_FRONTLEFT="";
  # Disable L515 to reduce fetch's CPU usage
  # Use L515 after finding a way to reduce L515 CPU usage like ConnectionBasedTransport
  # export RS_SERIAL_NO_L515_HEAD="f0211890";
  export RS_SERIAL_NO_L515_HEAD="";

  export NETWORK_DEFAULT_WIFI_INTERFACE="wlan0";
  export NETWORK_DEFAULT_ROS_INTERFACE="fetch15";
  # Comment out until Access point is back
  #export NETWORK_DEFAULT_PROFILE_ID="sanshiro-73B2";
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-outside";
elif [ $(hostname) = 'fetch1075' ]; then
  export DEFAULT_SPEAKER=2;
  export DEFAULT_ENGLISH_SPEAKER=cmu_us_slt.flitevox;
  export DEFAULT_WARNING_SPEAKER=cmu_us_lnh.flitevox;

  export USE_BASE_CAMERA_MOUNT=false;
  export USE_HEAD_BOX=false;
  export USE_HEAD_L515=true;
  export USE_INSTA360_STAND=true;

  export RS_SERIAL_NO_T265="";
  export RS_SERIAL_NO_D435_FRONTRIGHT="";
  export RS_SERIAL_NO_D435_FRONTLEFT="";
  # Disable L515 to reduce fetch's CPU usage
  # Use L515 after finding a way to reduce L515 CPU usage like ConnectionBasedTransport
  export RS_SERIAL_NO_L515_HEAD="f0232270";
  #export RS_SERIAL_NO_L515_HEAD="";
  # Currently, fetch1075 uses USB 2 for L515, so high resolution mode is not available.
  export L515_HIGH_RESOLUTION=false;

  export NETWORK_DEFAULT_WIFI_INTERFACE="wlan1";
  export NETWORK_DEFAULT_ROS_INTERFACE="fetch1075";
  # Comment out until Access point is back
  #export NETWORK_DEFAULT_PROFILE_ID="sanshiro-73B2";
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-outside";
fi
