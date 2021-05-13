# Getting Started with OpenVINS

## Building OpenVINS

Standalone OpenVINS compilation:

```
git clone https://github.com/ILLIXR/open_vins.git
cd open_vins
make [opt|dbg]
```

OpenVINS ILLIXR plugin compilation:

```
git clone https://github.com/ILLIXR/open_vins.git
cd open_vins
ILLIXR_INTEGRATION=yes make [opt|dbg]
#   or
export ILLIXR_INTEGRATION=yes
make [opt|dbg]
```

## Running OpenVINS Standalone

Now to run OpenVINS Standalone do the following:
```
# opt => <build-type> == RelWithDebInfo, dbg => <build-type> == Debug
cd build/<build-type>/ov_msckf
./run_illixr_msckf <path_to_cam0> <path_to_cam1> <path_to_imu0> <path_to_cam0_images> <path_to_cam1_images>
```

When running OpenVINS we assume the data is formatted in the [EUROC Dataset standard](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
