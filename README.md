# tt_pkg

工训上位机程序包

## Install

### Prerequisites

- Ubuntu20.04
- ROS2 Foxy

### Step

1.在用户目录下创建文件夹 `TransportTrolley/src` 并跳转
```
mkdir TransportTrolley && cd TransportTrolley
mkdir src && cd src
```
2.克隆本仓库
```
git clone https://github.com/CygnoidesV/tt_pkg.git
```

### Other

1.服务器端配有显示设备的前提下，使用如下命令允许用户远程控制服务器显示
```
export DISPLAY=:0 // 根据显示设备编号更换数字
xhost + 
```