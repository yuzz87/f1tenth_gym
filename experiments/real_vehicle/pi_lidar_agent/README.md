# Raspberry Pi RPLIDAR UDP Agent

This process uses the official SLAMTEC SDK without ROS 2 on the Raspberry Pi.
It converts each raw revolution to 360 non-duplicated one-degree bins and sends
three CRC-protected UDP packets to the PC.

## Build on the Pi

```bash
make -C ~/rplidar_sdk -j2
cd ~/real_vehicle_pi_lidar_agent
make -j2
```

## Synthetic check

```bash
./build/real_vehicle_lidar_agent \
  --synthetic \
  --host PC_IP_OR_HOSTNAME \
  --max-scans 20
```

## Real LiDAR

```bash
PC_LIDAR_HOST=PC_IP_OR_HOSTNAME ./run_lidar_agent.sh
```

The hardware direction check proved that the raw left/right convention is
opposite to ROS, so `run_lidar_agent.sh` enables `--angle-inverted` by default.
Set `LIDAR_ANGLE_INVERTED=false` only for a diagnostic comparison. Set
`LIDAR_ANGLE_OFFSET_RAD` after the physical zero direction is measured.

The service installer intentionally requires `PC_LIDAR_HOST`. Do not enable the
service at boot until manual startup, scan reception, and clean motor shutdown
have all been verified.
